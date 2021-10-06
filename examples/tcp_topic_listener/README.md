# uorb topic TCP listener

After running, use the following command to connect using netcat (TCP client):

```shell
stty -echo -icanon && nc localhost 10924
```

The -echo parameter of stty turns off command echo, and -icanon turns off the confirmation function.
