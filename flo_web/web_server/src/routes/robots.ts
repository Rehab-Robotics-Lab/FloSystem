import Router from 'express-promise-router';
import * as db from '../db';
import bcrypt from 'bcrypt';
import crypto from 'crypto';
import { checkLoggedIn, checkAdmin } from './users';
// create a new express-promise-router
// this has the same API as the normal express router except
// it allows you to use async functions as route handlers
const router = Router();
// export our router to be mounted by the parent application
export default router;

const saltRounds = 10;

router.post('/new-password', checkAdmin, async (req, res) => {
    const { robotName } = req.body;

    const passwordHex = await crypto.randomBytes(48);
    const password = passwordHex.toString('hex');

    let passwordHash;
    try {
        const salt = await bcrypt.genSalt(saltRounds);
        passwordHash = await bcrypt.hash(password, salt);
    } catch (e) {
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

    try {
        await db.query(
            'update robots set password_hash =$1 where robot_name=$2',
            [passwordHash, robotName],
        );
    } catch (e) {
        res.status(400).json({ error: e.toString() });
        return;
    }

    res.status(200).json({ newPassword: password });
});

router.post('/new-robot', checkAdmin, async (req, res) => {
    const { robotName, robotType } = req.body;

    const passwordHex = await crypto.randomBytes(48);
    const password = passwordHex.toString('hex');

    let passwordHash;
    try {
        const salt = await bcrypt.genSalt(saltRounds);
        passwordHash = await bcrypt.hash(password, salt);
    } catch (e) {
        res.status(500).json({ error: 'error when hashing new password' });
        return;
    }

    try {
        await db.query(
            'insert into robots (robot_name, password_hash, robot_type) ' +
                'values ($1,$2,(select id from robot_types where robot_type=$3))',
            [robotName, passwordHash, robotType],
        );
    } catch (e) {
        res.status(400).json({ error: e.toString() });
        return;
    }

    res.status(200).json({ newName: robotName, newPassword: password });
});

router.put('/ipaddr', async (req, res) => {
    const { name, password, ipaddr } = req.body;
    try {
        const {
            rows,
        } = await db.query(
            'select password_hash from robots where robot_name=$1',
            [name],
        );
        const validPassword = await bcrypt.compare(
            password,
            rows[0]['password_hash'],
        );

        if (!validPassword) {
            res.status(401).json({ error: 'failed to set ip address' });
            return;
        }

        await db.query('update robots set ipaddr=$1 where robot_name=$2', [
            ipaddr,
            name,
        ]);
    } catch {
        res.status(400).json({ error: 'failed to set ip address' });
        return;
    }
});

router.get('/all-robots', checkAdmin, async (req, res) => {
    try {
        const { rows } = await db.query(
            'select ' +
                'r.id, ' +
                'r.ipaddr, ' +
                'r.robot_name,' +
                'rt.robot_type, ' +
                'r.connected , ' +
                'r.battery, ' +
                'r.active_user_id, ' +
                'u.first_name active_user_first, ' +
                'u.last_name active_user_last, ' +
                'u.email active_user_email ' +
                'from robots r  ' +
                'left join users u on u.id = r.active_user_id ' +
                'left join robot_types rt on rt.id =r.robot_type ',
            [],
        );
        res.status(200).json({ robots: rows });
    } catch (e) {
        res.status(500).json({ error: e.toString() });
    }
});

// get api/robots/<id> Ex: api/robots/4
router.get('/:id', checkLoggedIn, async (req, res) => {
    const { id } = req.params;
    const { rows } = await db.query('SELECT * FROM robots WHERE id = $1', [id]);
    res.send(rows[0]);
});

router.get('/', checkLoggedIn, async (req, res) => {
    const userID = req.session!.userID;
    try {
        const {
            rows,
        } = await db.query(
            'select r.robot_name, r.connected , r.battery, r.active_user_id, u2.first_name active_user_first, u2.last_name active_user_last, rt.robot_type from users u ' +
                'inner join robot_permissions rp on rp.user_id=u.id ' +
                'inner join robots r on r.id=rp.robot_id ' +
                'left join users u2 on u2.id = r.active_user_id ' +
                'left join robot_types rt on rt.id=r.robot_type ' +
                'where u.id = $1;',
            [userID],
        );
        res.status(200).json({ robots: rows });
    } catch (e) {
        res.status(500).json({ error: e.toString() });
    }
});
