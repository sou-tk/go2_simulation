import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data

class GroundTruthTfBroadcaster(Node):
    def __init__(self):
        super().__init__('ground_truth_tf_broadcaster')
        
        # ★設定ポイント: 購読する真値トピック名
        # もし ros2 topic list で名前が違う場合はここを変えてください
        topic_name = '/odom/ground_truth'

        self.subscription = self.create_subscription(
            Odometry,
            topic_name,
            self.listener_callback,
            qos_profile_sensor_data)
        
        # TFを配信する機能 (Broadcaster)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(f'Start converting {topic_name} -> TF(odom)')

    def listener_callback(self, msg):
        t = TransformStamped()

        # 1. 時間の同期 (超重要)
        # Gazeboのシミュレーション時刻をそのままコピーします
        t.header.stamp = msg.header.stamp
        # ★ここを変更！「データの時刻」ではなく「現在時刻」を使います
        # これにより、常に「最新のTF」として扱われます
        #t.header.stamp = self.get_clock().now().to_msg()
        
        # 2. フレーム名の偽装 (なりすまし)
        # 真値は 'world' ですが、SLAMのために 'odom' だと名乗ります
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        #t.header.frame_id = 'test_odom'        # テスト用の親
        #t.child_frame_id = 'test_base_link'    # テスト用の子

        # 3. 座標データのコピー
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # 4. 回転データ(クォータニオン)のコピー
        t.transform.rotation = msg.pose.pose.orientation

        # 5. TFとして配信
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()