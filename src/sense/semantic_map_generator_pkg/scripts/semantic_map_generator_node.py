#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf
from grounding_sam_ros.client import SamDetector
from grounding_sam_ros.srv import VitDetection
from grounding_sam_ros.srv import UpdatePrompt, UpdatePromptResponse
import open3d as o3d
from threading import Lock, Thread
import time
import colorsys

class SemanticOctoMapGenerator:
    def __init__(self):
        rospy.init_node('semantic_octomap_generator_node', anonymous=True)
        
        # 初始化参数
        self.current_prompt = rospy.get_param("~default_prompt", "keyboard. mouse. cellphone. earphone. laptop. computer. water bottle. plant. keys. door. chair. ")
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_depth = None
        self.camera_info = None
        self.latest_pose = None
        
        # 初始化检测器
        self.detector = SamDetector()
        
        # 图像订阅
        image_sub = Subscriber("/camera/color/image_raw", Image)
        depth_sub = Subscriber("/camera/aligned_depth_to_color/image_raw", Image)

        # 相机内外参订阅
        camera_info_sub = Subscriber("/camera/color/camera_info", CameraInfo)
        pose_sub = Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped)

        # 同步订阅
        ts = ApproximateTimeSynchronizer([image_sub, depth_sub, camera_info_sub, pose_sub], queue_size=5, slop=0.1)
        ts.registerCallback(self.sync_sub_callback)

        # 标注图像发布
        self.annotated_pub = rospy.Publisher("~annotated", Image, queue_size=1)
        self.masks_pub = rospy.Publisher("~masks", Image, queue_size=1)

        # 语义地图发布
        self.semantic_cloud_pub = rospy.Publisher("~semantic_cloud", PointCloud2, queue_size=1)
        
        # Prompt更新服务
        rospy.Service("~update_prompt", UpdatePrompt, self.prompt_callback)
        
        # 定时器
        rospy.Timer(rospy.Duration(1), self.timer_callback)
        
        rospy.loginfo("Node initialization complete")

        # Open3D可视化初始化
        self.o3d_vis = o3d.visualization.Visualizer()
        self.o3d_vis.create_window(window_name='Semantic Map', width=1280, height=720)
        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
        self.o3d_vis.add_geometry(coord)
        
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.o3d_vis.add_geometry(self.o3d_pcd)
        self.pcd_lock = Lock()  # 线程安全锁
        
        # 启动独立可视化线程
        self.visualization_thread = Thread(target=self.update_visualization)
        self.visualization_thread.daemon = True
        self.visualization_thread.start()
        rospy.loginfo("Visualization thread started")

    def sync_sub_callback(self, img_msg, depth_msg, camera_info_msg, pose_msg):
        # 在此统一处理同步后的数据
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Image conversion failed: {str(e)}")
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except Exception as e:
            rospy.logerr(f"Depth conversion failed: {str(e)}")
        self.camera_info = camera_info_msg
        self.latest_pose = pose_msg

    def prompt_callback(self, req):
        """Prompt更新服务"""
        original_prompt = self.current_prompt  # 保留原值用于异常恢复
        
        try:
            # CASE 1: 直接赋值操作 =
            if req.data.startswith('='):
                new_prompt = req.data[1:].strip()  # 移除=号和首尾空格
                self.current_prompt = new_prompt
                rospy.loginfo(f"[Prompt SET] => {self.current_prompt}")
                return UpdatePromptResponse(
                    success=True,
                    message=f"Prompt SET: {self.current_prompt}"
                )

            # CASE 2: 追加操作 +
            elif req.data.startswith('+'):
                append_str = req.data[1:].strip()  # 移除+号和首尾空格
                
                # 空内容检查
                if not append_str:
                    rospy.logwarn("Empty append content")
                    return UpdatePromptResponse(
                        success=False,
                        message="Append content cannot be empty"
                    )
                
                # 智能空格拼接
                if self.current_prompt:
                    if not self.current_prompt.endswith(' '):
                        append_str = ' ' + append_str
                    self.current_prompt += append_str
                else:
                    self.current_prompt = append_str
                    
                rospy.loginfo(f"[Prompt APPEND] => {self.current_prompt}")
                return UpdatePromptResponse(
                    success=True,
                    message=f"Appended: {append_str}, current prompt: {self.current_prompt}"
                )

            # CASE 3: 非法操作
            else:
                rospy.logwarn(f"Illegal syntax: {req.data}")
                return UpdatePromptResponse(
                    success=False,
                    message="Invalid syntax. Start with '=' to SET or '+' to APPEND.\n"
                            "Example:\n"
                            "  = chair. table.\n"
                            "  + lamp. book."
                )

        except Exception as e:
            # 异常恢复机制
            self.current_prompt = original_prompt
            rospy.logerr(f"Prompt update failed: {str(e)}")
            return UpdatePromptResponse(
                success=False,
                message=f"Critical error: {str(e)}"
            )

    def apply_mask_overlay(self, image, masks):
        """将掩码以半透明固定颜色叠加到图像上"""
        overlay = image.copy()
        
        # 定义固定的颜色列表（RGB格式）
        fixed_colors = [
            [255, 0, 0],    # 红色
            [0, 255, 0],    # 绿色
            [0, 0, 255],    # 蓝色
            [255, 255, 0],  # 黄色
            [255, 0, 255],  # 紫色
            [0, 255, 255],  # 青色
            [128, 0, 0],    # 深红
            [0, 128, 0],    # 深绿
            [0, 0, 128],    # 深蓝
            [128, 128, 0],  # 橄榄色
        ]
        
        for i, mask in enumerate(masks):
            # 使用模运算循环选择颜色
            color = fixed_colors[i % len(fixed_colors)]
            
            # 将二值掩码转换为bool类型
            binary_mask = mask.astype(bool)
            
            # 创建颜色掩码
            color_mask = np.zeros_like(image)
            color_mask[binary_mask] = color
            
            # 使用cv2.addWeighted进行叠加
            alpha = 0.15  # 透明度
            cv2.addWeighted(color_mask, alpha, overlay, 1 - alpha, 0, overlay)
            
            # 绘制轮廓加强显示
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(overlay, contours, -1, color, 2)
        
        return overlay

    def pixel_to_world(self, u, v, z, cam_pose):
        """将像素坐标转换为世界坐标"""
        # 从CameraInfo获取内参
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]
        
        # 相机坐标系
        if z <= 0:  # 无效深度
            return None
        
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        # 转换为齐次坐标
        point_cam = np.array([x, y, z, 1])
        
        # 获取位姿变换矩阵
        pose = cam_pose.pose.pose
        rotation = np.array([pose.orientation.x, pose.orientation.y, 
                            pose.orientation.z, pose.orientation.w])
        translation = np.array([pose.position.x, pose.position.y, pose.position.z])
        
        # 构造变换矩阵fields
        T = np.eye(4)
        R = tf.transformations.quaternion_matrix(rotation)
        T[:3, :3] = R[:3, :3]
        T[:3, 3] = translation
        
        # 坐标变换
        point_world = T.dot(point_cam)
        return point_world[:3]

    def create_semantic_pointcloud(self, mask, label, score):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        
        points = []
        height, width = mask.shape
        
        # 降采样步长（根据性能调整）
        step = 2
        
        for v in range(0, height, step):
            for u in range(0, width, step):
                if mask[v, u] > 0:
                    z = self.latest_depth[v, u]
                    point = self.pixel_to_world(u, v, z, self.latest_pose)
                    if point is not None:
                        points.append([point[0], point[1], point[2], 
                                    hash(label) % 1000, score])  # 语义编码
                        
        # 创建PointCloud2消息
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('label', 12, PointField.UINT32, 1),
            PointField('confidence', 16, PointField.FLOAT32, 1)
        ]
        
        cloud = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=20,  # 4 * 5
            row_step=20*len(points)
        )
        
        cloud.data = np.array(points, np.float32).tobytes()
        rospy.loginfo(f"Generated {len(points)} points for {label}")
        return cloud

    def merge_clouds(self, cloud1, cloud2):
        # 实现点云合并逻辑
        merged = np.frombuffer(cloud1.data, np.float32).reshape(-1,5)
        merged = np.vstack((merged, 
                        np.frombuffer(cloud2.data, np.float32).reshape(-1,5)))
        
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('label', 12, PointField.UINT32, 1),
            PointField('confidence', 16, PointField.FLOAT32, 1)
        ]
        
        return PointCloud2(
            header=header,
            height=1,
            width=merged.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=20,
            row_step=20*merged.shape[0],
            data=merged.astype(np.float32).tobytes()
        )

    def timer_callback(self, event):
        """定时检测回调"""
        if self.latest_image is None:
            return

        try:
            # 执行检测
            # annotated_frame, boxes, masks, labels, scores
            annotated, boxes, masks, labels, scores = self.detector.detect(
                self.latest_image, 
                self.current_prompt
            )

            # 打印检测结果
            if len(labels) > 0:
                rospy.loginfo(f"Detected: {', '.join(labels)}")

            # 发布标注结果
            self.annotated_pub.publish(
                self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            )

            # 生成掩码叠加图像
            if len(masks) > 0:
                mask_overlay = self.apply_mask_overlay(annotated, masks)
                self.masks_pub.publish(
                    self.bridge.cv2_to_imgmsg(mask_overlay, "bgr8")
                )
                
        except rospy.ServiceException as e:
            rospy.logerr(f"Detection failed: {str(e)}")
        except Exception as e:
            rospy.logerr(f"Processing error: {str(e)}")
        
        # TODO
        # 生成语义点云
        semantic_clouds = []
        for mask, label, score in zip(masks, labels, scores):
            cloud = self.create_semantic_pointcloud(mask, label, score)
            semantic_clouds.append(cloud)

        # 合并点云发布
        if len(semantic_clouds) > 0:
            merged_cloud = semantic_clouds[0]
            for cloud in semantic_clouds[1:]:
                merged_cloud = self.merge_clouds(merged_cloud, cloud)
            self.semantic_cloud_pub.publish(merged_cloud)


        self.latest_image = None
        self.latest_depth = None
        self.latest_pose = None

        
        # 在发布点云后添加Open3D更新
        if len(semantic_clouds) > 0:
            # 合并所有点云数据
            all_points = []
            all_colors = []
            
            for cloud in semantic_clouds:
                try:
                    # 转换点云数据
                    arr = np.frombuffer(cloud.data, dtype=np.float32).reshape(-1,5)
                    points = arr[:, :3]
                    
                    # 数值有效性检查
                    valid_mask = ~np.isnan(points).any(axis=1)
                    points = points[valid_mask]
                    
                    if len(points) == 0:
                        continue
                    
                    # 打印坐标范围
                    rospy.loginfo(f"Point range - Min: {np.min(points, axis=0)} Max: {np.max(points, axis=0)}")
                    
                    # 颜色生成
                    labels = arr[valid_mask, 3].astype(int)
                    colors = np.array([colorsys.hsv_to_rgb((hash(l)%360)/360,0.8,0.8) for l in labels])
                    
                    all_points.append(points)
                    all_colors.append(colors)
                    
                except Exception as e:
                    rospy.logerr(f"Cloud processing error: {str(e)}")
            
            if len(all_points) > 0:
                merged_points = np.vstack(all_points)
                merged_colors = np.vstack(all_colors)
                
                # 更新Open3D显示
                with self.pcd_lock:
                    self.o3d_pcd.points = o3d.utility.Vector3dVector(merged_points)
                    self.o3d_pcd.colors = o3d.utility.Vector3dVector(merged_colors)
            else:
                rospy.logwarn("No valid points to display")

    def update_visualization(self):
        """独立可视化线程"""
        # 初始化视角参数
        ctr = self.o3d_vis.get_view_control()
        ctr.set_front([0, -1, 0])  # 相机朝向正Y方向
        ctr.set_up([0, 0, 1])     # 竖直向上为Z轴
        ctr.set_zoom(0.3)         # 初始缩放级别
        
        while True:
            try:
                with self.pcd_lock:
                    self.o3d_vis.update_geometry(self.o3d_pcd)
                
                # 强制重置视角
                if not hasattr(self, 'view_set'):
                    ctr = self.o3d_vis.get_view_control()
                    ctr.set_front([0, -1, 0])
                    ctr.set_up([0, 0, 1])
                    ctr.set_zoom(0.3)
                    self.view_set = True
                    
                # 控制渲染频率
                if not self.o3d_vis.poll_events():
                    break
                self.o3d_vis.update_renderer()
                time.sleep(0.05)
            except Exception as e:
                rospy.logerr(f"Visualization error: {str(e)}")
                break

if __name__ == '__main__':
    try:
        node = SemanticOctoMapGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass