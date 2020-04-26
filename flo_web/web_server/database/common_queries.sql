-- Finding out the user types of different users
select first_name, user_types.user_type from users 
inner join user_types on users.user_type = user_types.id;

    -- same thing but more terse
select first_name, ut.user_type from users u
inner join user_types ut on u.user_type = ut.id;

-- Adding users:
insert into users 
    (first_name, last_name, email, password_hash, user_type)
    values
    ('michael', 'sobrepera', 'mjsobrep@seas.upenn.edu', 'badpswd', (select id from user_types where user_type='administrator')),
    ('nick', 'hu', 'siyaohu@seas.upenn.edu', 'badpswd', (select id from user_types where user_type='standard'));

-- Adding robots
insert into robots 
    (robot_name, password_hash, robot_type)
    values
    ('lil''flo', 'badpswd', (select id from robot_types where robot_type='lilflo')),
    ('mantaro', 'badpswd', (select id from robot_types where robot_type='simple'));

-- Giving users permission
select id from robots where robot_name ='lil''flo';

select id from users where email='mjsobrep@seas.upenn.edu';

insert into robot_permissions 
    (user_id, robot_id)
    values
    ((select id from users where email='mjsobrep@seas.upenn.edu'),
     (select id from robots where robot_name='lil''flo'));
    
insert into robot_permissions 
    (user_id, robot_id)
    values
    ((select id from users where email='mjsobrep@seas.upenn.edu'),
     (select id from robots where robot_name='mantaro')),
    ((select id from users where email='siyaohu@seas.upenn.edu'),
     (select id from robots where robot_name='mantaro'));
 
-- Get list of permissions with robot and user names:
select r.robot_name , u.first_name, u.last_name, u.email from users u
inner join robot_permissions rp on rp.user_id=u.id
inner join robots r on r.id=rp.robot_id ;

-- get robots available to one user:
select r.robot_name , r.id robot_id from users u
inner join robot_permissions rp on rp.user_id=u.id
inner join robots r on r.id=rp.robot_id
where u.email = 'mjsobrep@seas.upenn.edu';

-- get login info for a user:
select password_hash, u.id, ut.user_type  from users u 
inner join user_types ut on u.user_type =ut.id 
where email ='mjsobrep@seas.upenn.edu'

-- set password:
update users set password_hash ='badpswd2' where email='mjsobrep@seas.upenn.edu'

-- Set a robot to be connected to a user:
update robots 
set active_user_id =(
    select id from users where email='mjsobrep@seas.upenn.edu')
where robot_name ='mantaro'

-- set robots as connected:
update robots set connected=true where robot_name in ('lil''flo','mantaro');

-- Get a status page for robots that a user has access to:
select r.robot_name , r.battery, r.active_user_id, u2.first_name active_user_first, u2.last_name active_user_last from users u
inner join robot_permissions rp on rp.user_id=u.id
inner join robots r on r.id=rp.robot_id
left join users u2 on u2.id = r.active_user_id 
where u.email = 'testuser@seas.upenn.edu'

-- Get the disconnected robots:
select robot_name, last_login 
from robots 
where connected =false;

--disconect a robot from a user
update robots set active_user_id =null where robot_name='lil''flo';

-- remove permissions
select * from robot_permissions

delete from robot_permissions
where robot_id =(select id from robots where robot_name='testrobot') and 
user_id=(select id from users where email = 'testuser@seas.upenn.edu');

