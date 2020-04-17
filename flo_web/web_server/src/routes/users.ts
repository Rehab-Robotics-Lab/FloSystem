import Router from 'express-promise-router';
import * as db from '../db';
import bcrypt from 'bcrypt';
import * as EmailValidator from 'email-validator';
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
    res.status(200).json(rows[0]);
});

router.post('/register', async (req, res) => {
    const { first_name, last_name, email, password } = req.body;
    let password_hash;
    const validEmail = EmailValidator.validate(email);
    if (!validEmail) {
        res.status(400).json({
            error: 'your email (' + email + ') is not valid',
        });
        return;
    }

    if (first_name === undefined) {
        res.status(400).json({ error: 'you must provide a first name' });
        return;
    }

    if (last_name === undefined) {
        res.status(400).json({ error: 'you must provide a last name' });
        return;
    }

    if (password === undefined || password.length < 5) {
        res.status(400).json({
            error: 'you must provide a password of at least 5 characters',
        });
        return;
    }

    try {
        const salt = await bcrypt.genSalt(saltRounds);
        password_hash = await bcrypt.hash(password, salt);
    } catch (e) {
        console.log('error when hashing new password: ' + e);
        res.status(500).json({ error: 'error when hashing new password' });
        return;
    }
    try {
        await db.query(
            'insert into users (first_name, last_name, email, password_hash) values ($1, $2, $3, $4)',
            [first_name, last_name, email, password_hash],
        );
    } catch (e) {
        if (e.code === '23505') {
            res.status(500).json({ error: 'user already exists' });
            return;
        }
        res.status(500).json({ error: e.detail });
    }

    res.status(200).json({ success: 'added user to system' });
});
