B) [optional] Change the security settings on the drone
https://github.com/daraosn/ardrone-wpa2 <-- change drone secuirty level


    Connect your laptop to your drone.
    Install with: script/install
    Connect to a network with: script/connect "<essid>" -p "<password>" -a <address> -d <droneip>

        $ cd home/turtlebot/BitSync/hast/Droneconfig/ardrone-wpa2-master
        $ . ./script/install
        $ . ./script/connect "router-ssid" -p "router-password" -a 192.168.1.200 -d 192.168.1.1
        $ telnet 192.168.1.200
        # route add default gw 192.168.1.1 ath0
        # wget -O - http://74.125.224.72/

        ### Change the launch file to:  <node pkg="ardrone_autonomy" type="ardrone_driver" name="ardrone_autonomy" output="screen" args="-ip 192.168.1.200" clear_params="true">

    Run to get help : . ./script/connect -h


### to continue using the ardrone in ros through the router, check out the launch file in this directory