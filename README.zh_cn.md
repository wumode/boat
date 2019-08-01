# Boat

### 依赖

必要:

- [cmake](http://www.cmake.org/) - an open-source, cross-platform family of tools designed to build, test and package software. 
- [serial](http://wjwwood.github.com/serial/) - Cross-platform, Serial Port library written in C++
- [eigen](http://eigen.tuxfamily.org/) - a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
- [glog](https://github.com/google/glog)  - a C++ implementation of the Google logging module. 

### 编译

clone:

`git clone git@github.com:wumode/boat.git`

build:

```shell
mkdir build
cd build
cmake ..
make
```

### 说明

#### 简介

无人船的主控程序：

- 自主航行
- 上位机
- 目标追踪
- 动力定位

#### 配置文件

配置文件: config.xml

参数说明：

- gps 自主航行预置航线
- mark 记录航线
  - flag
    - 0 不记录
    - 1 记录
  - path 保存文件名
  - period 记录周期（秒）
- boat 
  - frequency 控制频率
  - mode 运行模式
    - 1 遥控
    - 2 自主
    - 3 追踪
  - socket
    - send_frequency 发送频率
    - host 地址
    - port 端口
  - serial
    - prot 串口号
    - baud 波特率

### 运行

在boat目录下执行
`sudo build/./Ship`

### 日志
log/Ship.INFO

### 许可证

Copyright 2019 Sea Knight from HIT

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

​				 http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

### 作者

WuMo wumo1999@gmail.com

### Contact

https://www.westsite.cn:8001