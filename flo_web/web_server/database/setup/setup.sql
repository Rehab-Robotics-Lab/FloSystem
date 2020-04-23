create table user_types(
    id serial primary key,
    user_type text not null unique
);

insert into user_types
    (id, user_type) values (1,'standard'), (2,'administrator');

create table users(
    id serial primary key,
    first_name text not null,
    last_name text not null,
    email text not null unique,
    password_hash text not null,
    user_type int references user_types(id) default 1
);

create table robot_types(
    id serial primary key,
    robot_type text not null unique,
    description text  
);

insert into robot_types
    (robot_type, description)
    values
    ('lilflo', 'The complete lil''flo system with a humanoid on the telepresence system'),
    ('simple', 'A simple telepresence system with a single camera and the ability to drive');

create table robots(
    id serial primary key,
    robot_name text not null unique,
    password_hash text not null,
    ipaddr text,
    battery int,
    robot_type int references robot_types(id) not null,
    connected boolean not null default false,
    active_user_id int references users(id)
);

create table event_types(
    id serial primary key,
    event_type text not null unique
);

insert into event_types
    (event_type) values ('login'), ('logout'), ('connect'), ('disconnect');

create table user_events(
    id serial primary key,
    event_time timestamptz not null,
    event_type int references event_types(id) not null,
    user_id int references users(id) not null,
    target_robot_id int references robots(id)
);
    
create table robot_events(
    id serial primary key,
    event_time timestamptz,
    event_type int references event_types(id) not null,
    robot_id int references robots(id) not null,
    target_user_id int references users(id) 
);

create table robot_permissions(
    robot_id int references robots(id) not null, 
    user_id int references users(id) not null,
    primary key (user_id, robot_id)
);
        
