# Package 描述
&ensp;&ensp;这个package提供了一个使用camera一些的示例
&ensp;

# 文件目录

> camera_driver.py

```
提供了基于opencv2的camera识别蓝色物体的类CameraDriver
```

> example1_identify_color.py

```
CameraDriver类的使用示例
```

## opencv2

这个package使用了opencv2的库，安装参照一下连接

[https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)

## example1

* 接入uvc的usb摄像头，并确认系统已识别

```
// 一般dev目录下会存在video0、video1
$ ls -l /dev/video*

// 设置属性
$ sudo chmod +x /dev/video[xx]
```



* 启动编译源码

  参见首页readme的4.1~4.4

* 启动roscode

```
$ roscore
```

* 运行example1_identify_color.py

```
$ rosrun camera_demo example1_identify_color.py
```

会看到摄像头的原始视频流和识别到的蓝色物体

默认的程序是使用/dev/video0的摄像头，如果是其他设备号，需要修改example1_identify_color.py示例中"camera = CameraDriver(0)"的初始化设备号