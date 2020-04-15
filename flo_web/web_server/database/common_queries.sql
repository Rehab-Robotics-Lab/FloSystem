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
