# Web Server

## Postgres Database

For development:

Get the docker image of posgres

1. `docker pull postgres`
2. `docker run --rm --name pg-docker -e POSTGRES_PASSWORD=docker -e POSTGRES_DB=flodb -d -p 5432:5432 -v \$HOME/docker/volumes/postgres:/var/lib/postgresql/data postgres`
3. create a db: `createdb -h localhost -p 5432 -U postgres test`

See if you can get command line access:

3. `sudo apt install postgresql-client`
4. `psql -h localhost -U postgres`
    1. List databases with `\l`
    2. exit with `\q`
    3. `psql -h localhost -U postgres flodb`
    4. In theory you can do everything you want here

Get a manager setup:

6. `docker pull dpage/pgadmin4`
7. `docker run --rm --name pgadm-docker -e PGADMIN_DEFAULT_EMAIL=mjsobrep@live.com -e PGADMIN_DEFAULT_PASSWORD=docker -p 81:80 -d dpage/pgadmin4`
    1. navigate to localhost:81 and login with mjsobrep@live.com, docker
    2. add a connection named docker-local at localhost, port 5432,
       username postgres, password docker, and maintenance database flodb.
    3. Expand the local docker connection and databases
    4. Right click on flodb and click query tool. You can now type in queries
       and hit F5 to run the page.
    5. When a query runs, you can click to edit the data. Cool.

Try a different type of manager:

1. `sudo snap install dbeaver-ce`
2. `dbeaver-ce`
3. Enter the database details: host localhost, port 5432, database flodb, username
   postgres, password docker
4. Press F3 to open a sql editor.
5. Type in your sql commands. Press ctrl+enter to run the command you are hovering over
