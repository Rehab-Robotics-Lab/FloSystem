import Router from 'express-promise-router';
import * as db from '../db';
import bcrypt from 'bcrypt';
// create a new express-promise-router
// this has the same API as the normal express router except
// it allows you to use async functions as route handlers
const router = Router();
// export our router to be mounted by the parent application
export default router;

const saltRounds = 10;

router.get('/:id', async (req, res) => {
    const { id } = req.params;
    const { rows } = await db.query('SELECT * FROM users WHERE id = $1', [id]);
    res.send(rows[0]);
});

router.post('/register', async (req, res) => {
    const { first_name, last_name, email, password } = req.body;
    const salt = await bcrypt.genSalt(saltRounds);
    const password_hash = await bcrypt.hash(password, salt);
    await db.query(
        'insert into users (first_name, last_name, email, password_hash) values ($1, $2, $3, $4)',
        [first_name, last_name, email, password_hash],
    );
});
