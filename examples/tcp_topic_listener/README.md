# uorb topic TCP listener

uorb listener is a tcp server for uorb topic monitoring, which can monitor uorb topic data, uorb topic status, etc.

This is an example of uorb tcp listener. When the routine starts, we can use the tcp client to connect and send
commands.

The following description assumes that this example has been run and executed locally.

## View help

Support the help command, view all supported commands and a brief introduction.

### Command

```shell
echo "help" | nc 127.0.0.1 10924
```

### Example

```text
~ > echo "help" | nc -v 127.0.0.1 10924                 
localhost [127.0.0.1] 10924 open
Command list: 
        help: Print command list
        listener: topic listener, example: listener topic_name
        status: Print uorb status
        version: Print uorb version
```

## View the status of uorb

View the status of all topics in uorb, including the topic name of each topic, queue size, number of
publishers/subscribers, and the latest data publication index.

### Command

```shell
echo "status" | nc 127.0.0.1 10924
```

After the output, the tcp connection will be automatically disconnected.

### Example

```text
~ > echo "status" | nc -v 127.0.0.1 10924
localhost [127.0.0.1] 10924 open
topic                instance   queue      sub        pub        index
example_string       0          4          3+         3+         109
sensor_accel         0          1          3          4          29888
sensor_gyro          0          1          3          0          0
```

## Monitor real-time data on uorb topics

With the help of topic metadata, we can parse out the topic's binary data. The maximum monitoring frequency of topics is
10hz.

### Command

```shell
echo "listener topic_name" | nc 127.0.0.1 10924
```

The data will be printed to the terminal continuously, use `Ctrl+C` to terminate.

If you forget the topic name, you can first use the status command to view all topics and find the topic name you want.

### Example

```text
~ > echo "listener sensor_accel" | nc -v 127.0.0.1 10924
localhost [127.0.0.1] 10924 open
uint64_t timestamp = 604868299279
uint64_t timestamp_sample = 604868299279
uint32_t device_id = 10
float x = 998831.000000
float y = 1997662.000000
float z = 2996493.000000
float temperature = 3995324.000000
uint8_t[4] _padding0 = [0, 0, 0, 0]
...
```
