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
import struct
from grounding_sam_ros.client import SamDetector
from grounding_sam_ros.srv import VitDetection
from grounding_sam_ros.srv import UpdatePrompt, UpdatePromptResponse


class SemanticOctoMapGenerator:
    def __init__(self):
        rospy.init_node("semantic_octomap_generator_node")

        # 初始化参数
        self.current_prompt = rospy.get_param(
            "~default_prompt",
            "keyboard. mouse. cellphone. earphone. laptop. computer. water bottle. plant. keys. door. chair. ",
        )
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
        ts = ApproximateTimeSynchronizer(
            [image_sub, depth_sub, camera_info_sub, pose_sub], queue_size=5, slop=0.1
        )
        ts.registerCallback(self.sync_sub_callback)

        # 标注图像发布
        self.annotated_pub = rospy.Publisher("~annotated", Image, queue_size=1)
        self.masks_pub = rospy.Publisher("~masks", Image, queue_size=1)

        # 语义地图发布
        self.semantic_cloud_pub = rospy.Publisher(
            "~semantic_cloud", PointCloud2, queue_size=1
        )

        # Prompt更新服务
        rospy.Service("~update_prompt", UpdatePrompt, self.prompt_callback)

        # 定时器
        rospy.Timer(rospy.Duration(1), self.timer_callback)

        rospy.loginfo("Node initialization complete")

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
            if req.data.startswith("="):
                new_prompt = req.data[1:].strip()  # 移除=号和首尾空格
                self.current_prompt = new_prompt
                rospy.loginfo(f"[Prompt SET] => {self.current_prompt}")
                return UpdatePromptResponse(
                    success=True, message=f"Prompt SET: {self.current_prompt}"
                )

            # CASE 2: 追加操作 +
            elif req.data.startswith("+"):
                append_str = req.data[1:].strip()  # 移除+号和首尾空格

                # 空内容检查
                if not append_str:
                    rospy.logwarn("Empty append content")
                    return UpdatePromptResponse(
                        success=False, message="Append content cannot be empty"
                    )

                # 智能空格拼接
                if self.current_prompt:
                    if not self.current_prompt.endswith(" "):
                        append_str = " " + append_str
                    self.current_prompt += append_str
                else:
                    self.current_prompt = append_str

                rospy.loginfo(f"[Prompt APPEND] => {self.current_prompt}")
                return UpdatePromptResponse(
                    success=True,
                    message=f"Appended: {append_str}, current prompt: {self.current_prompt}",
                )

            # CASE 3: 非法操作
            else:
                rospy.logwarn(f"Illegal syntax: {req.data}")
                return UpdatePromptResponse(
                    success=False,
                    message="Invalid syntax. Start with '=' to SET or '+' to APPEND.\n"
                    "Example:\n"
                    "  = chair. table.\n"
                    "  + lamp. book.",
                )

        except Exception as e:
            # 异常恢复机制
            self.current_prompt = original_prompt
            rospy.logerr(f"Prompt update failed: {str(e)}")
            return UpdatePromptResponse(
                success=False, message=f"Critical error: {str(e)}"
            )

    def apply_mask_overlay(self, image, masks):
        """将掩码以半透明固定颜色叠加到图像上"""
        overlay = image.copy()

        # 定义固定的颜色列表（RGB格式）
        fixed_colors = [
            [255, 0, 0],  # 红色
            [0, 255, 0],  # 绿色
            [0, 0, 255],  # 蓝色
            [255, 255, 0],  # 黄色
            [255, 0, 255],  # 紫色
            [0, 255, 255],  # 青色
            [128, 0, 0],  # 深红
            [0, 128, 0],  # 深绿
            [0, 0, 128],  # 深蓝
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
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
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
        z = z / 1000.0  # mm -> m
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # 转换为齐次坐标
        point_cam = np.array([x, y, z, 1])

        # 获取位姿变换矩阵
        pose = cam_pose.pose.pose
        rotation = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
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
        # 创建点云头部信息
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        points = []
        height, width = mask.shape

        # 降采样步长（根据性能调整）
        step = 2

        # 遍历掩码区域生成点云
        for v in range(0, height, step):
            for u in range(0, width, step):
                if mask[v, u] > 0:  # 仅处理掩码有效区域
                    # 获取深度值并转换到世界坐标系
                    z = self.latest_depth[v, u]
                    point = self.pixel_to_world(u, v, z, self.latest_pose)

                    if point is not None:
                        # 提取颜色信息（BGR格式）
                        if (
                            0 <= v < self.latest_image.shape[0]
                            and 0 <= u < self.latest_image.shape[1]
                        ):
                            b, g, r = self.latest_image[v, u]
                        else:
                            r, g, b = 255, 255, 255  # 默认白色

                        # 将颜色打包为UINT32（0x00RRGGBB格式）
                        rgba_bytes = struct.pack("BBBB", b, g, r, 0)
                        rgba_value = struct.unpack("<I", rgba_bytes)[0]  # 小端解包

                        # 生成标签哈希（确保非负）
                        label_hash = abs(hash(label)) % 10000

                        # 添加点数据
                        points.append(
                            (
                                float(point[0]),  # x
                                float(point[1]),  # y
                                float(point[2]),  # z
                                rgba_value,  # 颜色
                                label_hash,  # 标签
                                float(score),  # 置信度
                            )
                        )

        breakpoint()

        # 定义点云字段结构
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgba", 12, PointField.UINT32, 1),
            PointField("label", 16, PointField.UINT32, 1),
            PointField("confidence", 20, PointField.FLOAT32, 1),
        ]

        # 创建点云消息
        cloud = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=False,  # 允许存在无效点
            is_bigendian=False,  # 使用小端字节序
            fields=fields,
            point_step=24,  # 6个字段×4字节=24
            row_step=24 * len(points),
        )

        # 使用numpy结构化数组高效打包数据
        if len(points) > 0:
            dtype = np.dtype(
                [
                    ("x", "<f4"),
                    ("y", "<f4"),
                    ("z", "<f4"),
                    ("rgba", "<u4"),
                    ("label", "<u4"),
                    ("confidence", "<f4"),
                ]
            )
            data_array = np.array(points, dtype=dtype)
            cloud.data = data_array.tobytes()

        rospy.loginfo(f"Generated semantic cloud: {len(points)} points [{label}]")
        return cloud

    # def create_semantic_pointcloud(self, mask, label, score):
    #     # 针对一张语义掩码生成语义点云
    #     header = Header()
    #     header.stamp = rospy.Time.now()
    #     header.frame_id = "map"

    #     points_cnt = 0
    #     packed_data = []
    #     height, width = mask.shape

    #     # 降采样步长（根据性能调整）
    #     step = 2

    #     for v in range(0, height, step):  # 针对掩码中的每个像素
    #         for u in range(0, width, step):
    #             if mask[v, u] > 0:
    #                 z = self.latest_depth[v, u]  # mm
    #                 point = self.pixel_to_world(u, v, z, self.latest_pose)  # m, 针对每个世界点
    #                 if point is not None:
    #                     points_cnt += 1
    #                     if 0 <= v < self.latest_image.shape[0] and 0 <= u < self.latest_image.shape[1]:
    #                         b, g, r = self.latest_image[v, u]  # 提取BGR
    #                     else:
    #                         r, g, b = 255, 255, 255  # 默认白色
    #                     # 将RGB打包成UINT32（格式：0x00RRGGBB）
    #                     rgba = struct.pack('BBBB', b, g, r, 0)
    #                     rgb_value = struct.unpack('<I', rgba)[0]
    #                     # 生成标签哈希（确保非负)
    #                     label_hash = abs(hash(label)) % 10000
    #                     # 打包点数据:x,y,z,rgb_value,label,score
    #                     packed_data.append(struct.pack('<fffIIf', point[0], point[1], point[2], rgb_value, label_hash, score))

    #     # 创建PointCloud2消息
    #     fields = [
    #         PointField('x', 0, PointField.FLOAT32, 1),
    #         PointField('y', 4, PointField.FLOAT32, 1),
    #         PointField('z', 8, PointField.FLOAT32, 1),
    #         PointField('rgba', 12, PointField.UINT32, 1),  # 颜色通道
    #         PointField('label', 16, PointField.UINT32, 1),
    #         PointField('confidence', 20, PointField.FLOAT32, 1)
    #     ]

    #     cloud = PointCloud2(
    #         header=header,
    #         height=1,
    #         width=points_cnt,
    #         is_dense=False,
    #         is_bigendian=False,
    #         fields=fields,
    #         point_step=24,  # 4 * 6 = 24字节/点
    #         row_step=24*points_cnt,
    #         data=b''.join(packed_data)
    #     )

    #     rospy.loginfo(f"Generated {points_cnt} points for {label}")
    #     return cloud

    # def merge_clouds(self, cloud1, cloud2):
    #     """合并两个包含6个字段的点云（x,y,z,rgba,label,confidence）"""
    #     # 解析第一个点云
    #     dtype1 = np.dtype([
    #         ('x', np.float32),
    #         ('y', np.float32),
    #         ('z', np.float32),
    #         ('rgba', np.uint32),
    #         ('label', np.uint32),
    #         ('confidence', np.float32)
    #     ])
    #     arr1 = np.frombuffer(cloud1.data, dtype=dtype1)

    #     # 解析第二个点云
    #     dtype2 = np.dtype([
    #         ('x', np.float32),
    #         ('y', np.float32),
    #         ('z', np.float32),
    #         ('rgba', np.uint32),
    #         ('label', np.uint32),
    #         ('confidence', np.float32)
    #     ])
    #     arr2 = np.frombuffer(cloud2.data, dtype=dtype2)

    #     # 合并数组
    #     merged_arr = np.hstack((arr1, arr2))

    #     # 创建新点云
    #     header = Header()
    #     header.stamp = rospy.Time.now()
    #     header.frame_id = "map"

    #     fields = [
    #         PointField('x', 0, PointField.FLOAT32, 1),
    #         PointField('y', 4, PointField.FLOAT32, 1),
    #         PointField('z', 8, PointField.FLOAT32, 1),
    #         PointField('rgba', 12, PointField.UINT32, 1),
    #         PointField('label', 16, PointField.UINT32, 1),
    #         PointField('confidence', 20, PointField.FLOAT32, 1)
    #     ]

    #     return PointCloud2(
    #         header=header,
    #         height=1,
    #         width=len(merged_arr),
    #         is_dense=False,
    #         is_bigendian=False,
    #         fields=fields,
    #         point_step=24,  # 6个字段 * 4字节 = 24
    #         row_step=24 * len(merged_arr),
    #         data=merged_arr.tobytes()
    #     )

    def timer_callback(self, event):
        """定时检测回调"""
        if self.latest_image is None:
            return

        try:
            # 执行检测
            # annotated_frame, boxes, masks, labels, scores
            annotated, boxes, masks, labels, scores = self.detector.detect(
                self.latest_image, self.current_prompt
            )

            # 打印检测结果
            if len(labels) > 0:
                rospy.loginfo(f"Detected: {', '.join(labels)}")

            # 发布标注结果
            self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))

            # 生成掩码叠加图像
            if len(masks) > 0:
                mask_overlay = self.apply_mask_overlay(annotated, masks)
                self.masks_pub.publish(self.bridge.cv2_to_imgmsg(mask_overlay, "bgr8"))

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

            self.semantic_cloud_pub.publish(cloud)

        # # 合并点云发布
        # if len(semantic_clouds) > 0:
        #     merged_cloud = semantic_clouds[0]
        #     for cloud in semantic_clouds[1:]:
        #         merged_cloud = self.merge_clouds(merged_cloud, cloud)
        #     self.semantic_cloud_pub.publish(merged_cloud)

        self.latest_image = None
        self.latest_depth = None
        self.latest_pose = None


if __name__ == "__main__":
    try:
        node = SemanticOctoMapGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
