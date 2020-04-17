import Router from 'express-promise-router';
import * as db from '../db';
import bcrypt from 'bcrypt';
import crypto from 'crypto';
// create a new express-promise-router
// this has the same API as the normal express router except
// it allows you to use async functions as route handlers
const router = Router();
// export our router to be mounted by the parent application
export default router;

const saltRounds = 10;

router.get('/new-password', async (req, res) => {
    const { robotName } = req.body;

    console.log('generating password');
    const passwordHex = await crypto.randomBytes(48);
    const password = passwordHex.toString('hex');

    console.log('hashing password');
    let passwordHash;
    try {
        const salt = await bcrypt.genSalt(saltRounds);
        passwordHash = await bcrypt.hash(password, salt);
    } catch (e) {
        console.log('error when hashing new password: ' + e);
        res.status(500).json({ error: 'error when hashing new password' });
        return;
    }

    try {
        const {
            rows,
        } = await db.query('select count(*) from robots where robot_name=$1', [
            robotName,
        ]);
        if (rows[0].count < 1) {
            res.status(400).json({
                error: 'the specified robot could not be found',
            });
            return;
        }
    } catch (e) {
        res.status(400).json({
            error: 'the specified robot could not be found',
        });
        return;
    }

    console.log('inserting password');
    try {
        await db.query(
            'update robots set password_hash =$1 where robot_name=$2',
            [passwordHash, robotName],
        );
    } catch (e) {
        console.log('error: ' + e);
        res.status(400).json({ error: e });
        return;
    }

    res.status(200).json({ newPassword: password });
});

// get api/robots/<id> Ex: api/robots/4
router.get('/:id', async (req, res) => {
    const { id } = req.params;
    const { rows } = await db.query('SELECT * FROM robots WHERE id = $1', [id]);
    res.send(rows[0]);
});

router.get('/', async (req, res) => {
    const { rows } = await db.query('select * from robots', []);
    res.send(rows);
});
