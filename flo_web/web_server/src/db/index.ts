import { Pool } from 'pg';

//default pg.client() or pg.pool() connect values:
//PGHOST='localhost'
//PGUSER=process.env.USER
//PGDATABASE=process.env.USER
//PGPASSWORD=null
//PGPORT=5432
// TODO: set environment variables
const pool = new Pool({
    user: 'postgres',
    host: 'postgres',
    database: 'flodb',
    password: 'docker',
});

const query = (text: string, params: Array<any>) => pool.query(text, params);

export { query };
