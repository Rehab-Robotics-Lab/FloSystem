-- This file is a template config file for lsyncd for working with the robot.
-- You should copy this file and rename it to mySyncConfig.lua, which is already in
-- the gitignore and will therefore be ignored by source control. You can then
-- edit it for your needs. It can then be run with `lsyncd <this file>`
-- You will need to setup ssh keys if you want this to work without constantly
-- entering passwords. You can do that by running (on your development machine):
--     ssh-keygen -t rsa -f ~/.ssh/id_rsa
--     ssh flo-username@flo-ip 'mkdir -p .ssh; touch .ssh/authorized_keys; cat >> .ssh/authorized_keys' < ~/.ssh/id_rsa.pub
-- Note, you can leave your rsa keys with no passphrase, but you might want some
-- more security: https://superuser.com/questions/261361/do-i-need-to-have-a-passphrase-for-my-ssh-rsa-key

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
    exclude = {'/.git','node_modules'},
    rsync     = {
        archive  = true,
        compress = true
    }
}
