#!/bin/bash
#!/bin/bash
{
    gnome-terminal -t "pgm_pub" -- bash -c "/home/iee_gjm/out/Shell/pgm_pub.sh;exec bash"
}&
sleep 2s
{
    gnome-terminal -t "yaml_pub" -- bash -c "/home/iee_gjm/out/Shell/yaml_pub.sh;exec bash"
}
