import serial
import yaml
import subprocess
import rospy
import std_srvs

from std_srvs.srv import SetBool
#发送特定字符串给单片机，激发蜂鸣器
def buzzer():
    print("do sth")

def init_Uart(Baudrate=115200,dev='/dev/ttyTHS0'):
    subprocess.call('echo "123456" | sudo -S chmod 777 /dev/ttyTHS0', shell=True)
    ser = serial.Serial(dev, Baudrate)
    if ser.isOpen == False:       
        ser.open()                
    ser.flushInput()
    return ser


#rospy.init_node("Mission_Controller")
#rospy.wait_for_service("/mission/boot")
#boot_client = rospy.ServiceProxy("/mission/boot",SetBool)

# resp = boot_client(True)
# print("Mission Boot Call Success!")
# resp = boot_client(True)
# print(resp)
# rospy.set_param("/mission_param",[0.0,0.0,0.0,0.0])
ser = init_Uart(Baudrate=115200,dev='/dev/ttyTHS0')
ser.write('1'.encode())

accept = ''
while 1:
    
    while accept == '':
        if(accept == ''):
            print('waiting...')
        else:
            print('erorr!!!')
        accept = ser.read(1)
        # accept = ser.readline()
        # accept = accept.decode("utf-8")
        print(accept)
        if(accept ==b'\x01'):
            # resp = boot_client(True)
            print("Mission Boot Call Success!")
            accept = ''
            data=[0xBB,0x01,0x02,0x01,0x01,0x01,0xCC]
            
            byte=bytes(data)
            ser.write(byte)
            print(byte)
        elif(accept == b'\x02'):
                print("Mission 1 Set Success!")
                accept = ''
        else:# 收到的数据不符合格式清空 继续接收
                print("Unkown Command!")
                accept = ''
            






