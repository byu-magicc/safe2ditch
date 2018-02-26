Safe2Ditch MAVProxy Modules
===========================

Because of the way the module loading scheme of `mavproxy.py` works, the files within this local `modules` directory must be within the `modules` directory that is in the `/usr/local/lib/python2.7/dist-packages/MAVProxy/modules` directory (or equivalent, based on how you installed mavproxy). To accomodate mavproxy, but still maintain source control over these files, "install" them by creating symlinks:

```bash
$ sudo updatedb && locate -i mavproxy # find out where mavproxy is installed
$ cd /usr/local/lib/python2.7/dist-packages/MAVProxy/modules
$ sudo ln -s $HOME/dev/safe2ditch/mavproxy/modules/mavproxy_safe2ditch.py
$ sudo ln -s $HOME/dev/safe2ditch/mavproxy/modules/safe2ditch_source
```
