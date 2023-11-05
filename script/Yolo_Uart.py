import cv2
import rospy
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import threading
from Uart_and_yaml import write_yaml,read_yaml,start_uart,clean_yaml

#选择classify or detect模式
mode = "detect" 
# mode ="classify"

# subprocess.call("/home/drone/Public/lidar.bash",shell=True)

bridge=CvBridge()

target_msg = Float32MultiArray(data=[0.0,0.0,0.0,0.0,0.0])
classify_msg = Float32(0.0)

#可用于设置回调函数是否运行 
callbackflag = 1

yaml_data = {
"state": 0 ,
"po1": 0,
"po2": 0,
"number": 0
}

# 多线程 单片机通信和yaml文件读写部分
def thread_uart_yaml():
    global yaml_data
    yaml_data = read_yaml()
    start_uart()

#YOLO检测部分
#ros回调函数
def callback(ros_img_msg ):
    global callbackflag
    if( callbackflag == 1):
        # ros图像转化成cv格式
        opencv_image = bridge.imgmsg_to_cv2(ros_img_msg,"bgr8")
        # cv2.imshow("convert", opencv_image)
        results = model.predict(opencv_image , show = False)
        opencv_image = results[0].plot()
        cv2.imshow("YOLOv8 Inference", opencv_image)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
    else:
        print("callback_paused")
    rate.sleep()
   

#模型回调函数，主要修改此处即可
def object_callback(predictor):
    name = predictor.results[0].names
    if mode == "detect":
        boxes = predictor.results[0].boxes
        for box in boxes:
            #返回Box类别名称,box.cls.tolist()[0]为类别序号
            # print(name.get(box.cls.tolist()[0]))
            if(box.cls.tolist()[0] == 0):
                #发送数据到话题 数据类型为float32数组 前四位为中心点xy与长宽wh，最后一位为类别编号
                temp = box.xywh.tolist()[0]
                temp.append(box.cls.tolist()[0])
                # print(temp)
                target_msg = Float32MultiArray(data=temp)
                result_pub.publish(target_msg)
            
    elif mode == "classify":
        probs = predictor.results[0].probs
        if(probs.data[probs.top1]>=0.5):
            print("detect "+name.get(probs.top1))
            #发送数据到话题 数据类型为float32 分类概率第一的类别编号
            classify_msg = Float32(probs.top1)
            result_pub.publish(classify_msg)
            #可以设置在检测到目标之后暂停检测
            # if( name.get(probs.top1) == "goldfish" ):
                # global callbackflag
                # callbackflag = 0
                # # 也可以直接注销订阅节点
                # # img_sub.unregister()

# 后续初始化较久，先开启Uart线程
# add_thread = threading.Thread(target = thread_uart_yaml)
# add_thread.start()

#根据任务选择模型
if mode == "classify":
    model = YOLO('/home/drone/ultralytics/yolov8n-cls.pt')
elif mode == "detect":
    # model =YOLO('/home/drone/ultralytics/yolov8n.pt')
    model = YOLO('best1.engine',task='detect')
#添加预测后的回调函数
model.add_callback("on_predict_end", object_callback)

rospy.init_node('Yolo_node', anonymous=True)
rate = rospy.Rate(20)
#ros回调方式一直进行检测
img_sub=rospy.Subscriber("/camera/color/image_raw", Image, callback , queue_size=5)

#可通过ros参数服务器通信来设置任务参数
# rospy.set_param(param_name='/mission_param', param_value = [0,0,0,0,0])
# rospy.set_param(param_name='/mission_', param_value = 0 )


#ros话题方式进行实时性结果参数传递
if mode == "detect":
    result_pub = rospy.Publisher("/yolo_result",Float32MultiArray, queue_size=2)
elif mode == "classify":
    result_pub = rospy.Publisher("/yolo_result",Float32, queue_size=2)

#不想采用ros spin回调时，可以用下列方法控制回调函数次数，减少程序资源占用
# while 1:
#     rosimg = rospy.wait_for_message("/camera/color/image_raw", Image)
#     callback(rosimg)
#     result_pub = rospy.Publisher("/yolo_result",Float32MultiArray, queue_size=1)
#     result_pub.publish(target_msg)
#     rate.sleep()

#保持程序不结束
rospy.spin()
cv2.destroyAllWindows()

# 不使用ROS获取图片 直接用USB摄像头获取时可以使用以下代码
# cap = cv2.VideoCapture(0)
# while cap.isOpened():
    # success, frame = cap.read()
    # if success:
        # 直接调用回调函数检测，此时需要修改callback 不进行图像类型转换
        # callback(frame)
        # sleep(0.08)
    # else:
    #     # Break the loop if the end of the video is reached
    #     break
# Release the video capture object and close the display window
# cap.release()


