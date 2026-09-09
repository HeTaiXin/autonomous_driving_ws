import rclpy
from rclpy.node import Node  # 导入 rclpy 文件夹下的 node.py 文件的 Node 函数
import numpy as np
import matplotlib.pyplot as plt

class PlotData(Node):  # PlotData 继承自 Node
    def __init__(self, node_name:str, file_name:str):
        super().__init__(node_name)
        self.file_name = file_name
    
    def dataPlot(self):
        print(f'Hi from {self.file_name}')
        self.get_logger().info(f'Hi from {self.file_name}')

def main():
    rclpy.init()
    node = PlotData("data_plot_node", "data plot")
    node.dataPlot()
    node.get_logger().info('测试内容')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Interrupted by user')
    finally:
        rclpy.shutdown()  # 防止按 ctrl + c 时无法关闭

if __name__ == '__main__':
    main()
