-- This file is a template config file for lsyncd for working with the robot. 
-- You should copy this file and rename it to mySyncConfig, which is already in 
-- the gitignore and will therefore be ignored by source control. You can then
-- edit it for your needs. It can then be run with `lsyncd <this file>`

settings{
    nodaemon = true -- We want to be able to see this run and turn it off.
}
sync {
    default.rsync,
    source    = "/home/mjsobrep/Documents/git/LilFloSystem",
    target    = "flo@192.168.1.10:/home/flo/catkin_ws/src/LilFloSystem",
    delay     = 1, -- How long to wait before executing. Give it a second in 
                   -- case there are lots of changes we can put together, but 
                   -- don't want to be sitting around forever... 
    exclude = {'/.git'},
    rsync     = {
        archive  = true,
        compress = true
    }
}