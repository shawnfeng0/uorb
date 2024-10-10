# uORB

![Tests](https://github.com/ShawnFeng0/uorb/actions/workflows/tests.yml/badge.svg)

uORB是用于线程间通信的消息发布/订阅的库。"UORB"是Micro Object Request Broker的缩写。

uORB源自PX4开源飞控的消息中间件，运行在Nuttx rtos和Linux上：

* [PX4中的uORB介绍](https://dev.px4.io/master/en/middleware/uorb.html)

这个项目重新在原始API的上基于POSIX重新实现了，去除了一些冗余的软件层，并且保留了大部分核心功能。
其中一些改进和修复已经合并到PX4上游。

* [与PX4的uORB的对比](docs/comparison_with_px4_uorb.md)

## 特性

* 基于POSIX，有良好的兼容性
* 类似于poll()函数的实时响应机制
* 灵活和易用的C++接口，就像操作本地数据一样操作订阅数据

## 依赖关系

编译uORB库需要C++11的支持，目前大多数编译器都支持。

需要一个消息生成器用来生成消息元数据，需要这些python库才能工作：

```shell
pip3 install -r tools/msg/tools/requirements.txt
```

## 文档

* [入门指导](docs/getting_started_cn.md)
* API 参考 (TODO)
* [变更日志](CHANGELOG.md)

## 示例

[uorb-examples](https://github.com/ShawnFeng0/uorb-examples.git)

## 工具

### uorb 主题监听器

uorb 还有一个[话题监听器库](tools/uorb_tcp_topic_listener_lib)。 它负责启动一个tcp服务器，方便开发者在进程外实时监控uorb话题数据。

这里有一个使用监听器的[示例](examples/tcp_topic_listener)。
