import Router from 'express-promise-router';
import * as db from '../db';
import bcrypt from 'bcrypt';
import * as EmailValidator from 'email-validator';
import express from 'express';
import crypto from 'crypto';
// create a new express-promise-router
// this has the same API as the normal express router except
// it allows you to use async functions as route handlers
const router = Router();
// export our router to be mounted by the parent application
export default router;

const saltRounds = 10;

export const checkLoggedIn: express.RequestHandler = (req, res, next) => {
    if (req.session!.userID === undefined) {
        res.status(400).json({ error: 'you are not logged in' });
        return;
    }
    next();
};

const checkLoggedOut: express.RequestHandler = (req, res, next) => {
    if (req.session!.userID !== undefined) {
        res.status(400).json({ error: 'you are already logged in' });
        return;
    }
    next();
};

export const checkAdmin: express.RequestHandler = (req, res, next) => {
    if (req.session!.userID === undefined) {
        res.status(400).json({ error: 'you are not logged in' });
        return;
    }
    if (req.session!.userType !== 'administrator') {
        res.status(400).json({ error: 'you are not authorized' });
        return;
    }
    next();
};

router.get('/login', async (req, res) => {
    if (req.session!.userID === undefined) {
        res.status(200).json({ loggedIn: false });
        return;
    }
    res.status(200).json({
        loggedIn: true,
        userName: req.session!.userName,
        userType: req.session!.userType,
    });
});

router.post('/login', checkLoggedOut, async (req, res) => {
    const { email, password } = req.body;
    const emailLC = email.toLowerCase();
    try {
        const {
            rows,
        } = await db.query(
            'select password_hash, u.id, ut.user_type, u.first_name, u.last_name  from users u' +
                ' inner join user_types ut on u.user_type =ut.id' +
                ' where email =$1',
            [emailLC],
        );
        const passwordHash = rows[0]['password_hash'];
        const validPassword = await bcrypt.compare(password, passwordHash);
        if (!validPassword) {
            res.status(401).json({ error: 'invalid password' });
            return;
        }
        req.session!.userID = rows[0]['id'];
        const userType = rows[0]['user_type'];
        req.session!.userType = userType;
        const userName = rows[0]['first_name'] + ' ' + rows[0]['last_name'];
        req.session!.userName = userName;
        console.log(req.session);
        res.status(200).json({
            userName: userName,
            userType: userType,
            success: 'succesfully logged in',
        });
        return;
    } catch (e) {
        res.status(401).json({ error: e.toString() });
        return;
    }
});

router.post('/logout', checkLoggedIn, async (req, res) => {
    req.session!.destroy((err) => {
        res.clearCookie('connect.sid');

        if (err) {
            res.status(500).json({ error: 'something went wrong' });
            return;
        }
        res.status(200).json({ success: 'logged out' });
    });
});

router.post('/register', checkLoggedOut, async (req, res) => {
    const { firstName, lastName, email, password } = req.body;
    const emailLC = email.toLowerCase();
    let passwordHash;
    const validEmail = EmailValidator.validate(emailLC);
    if (!validEmail) {
        res.status(400).json({
            error: 'your email (' + email + ') is not valid',
        });
        return;
    }

    if (firstName === undefined) {
        res.status(400).json({ error: 'you must provide a first name' });
        return;
    }

    if (lastName === undefined) {
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
        passwordHash = await bcrypt.hash(password, salt);
    } catch (e) {
        res.status(500).json({ error: 'error when hashing new password' });
        return;
    }
    try {
        await db.query(
            'insert into users (first_name, last_name, email, password_hash) values ($1, $2, $3, $4)',
            [firstName, lastName, emailLC, passwordHash],
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

router.get('/all-users', checkAdmin, async (req, res) => {
    try {
        const { rows } = await db.query(
            'select u.id, u.email, u.first_name, u.last_name,  ut.user_type from users u ' +
                'inner join user_types ut on ut.id = u.user_type ',
            [],
        );
        res.status(200).json({ users: rows });
    } catch (e) {
        res.status(500).json({ error: e.toString() });
    }
});

router.post('/change-password', checkLoggedIn, async (req, res) => {
    const { oldPassword, newPassword } = req.body;
    try {
        const {
            rows,
        } = await db.query('select password_hash from users where id =$1', [
            req.session!.userID,
        ]);
        const passwordHash = rows[0]['password_hash'];
        const validPassword = await bcrypt.compare(oldPassword, passwordHash);
        if (!validPassword) {
            res.status(401).json({ error: 'invalid password' });
            return;
        }
        const salt = await bcrypt.genSalt(saltRounds);
        const newPasswordHash = await bcrypt.hash(newPassword, salt);
        await db.query('update users set password_hash =$1 where id=$2', [
            newPasswordHash,
            req.session!.userID,
        ]);
        res.status(200).json({ success: 'Password succesfully changed' });
    } catch (e) {
        res.status(500).json({
            error: e.toString(),
        });
    }
});

router.post('/change-type', checkAdmin, async (req, res) => {
    const { email, userType } = req.body;
    try {
        await db.query(
            'update users set user_type =(select id from user_types where user_type=$1) where email=$2',
            [userType, email],
        );
        res.status(200).json({ success: 'User type succesfully changed' });
    } catch (e) {
        res.status(500).json({
            error: e.toString(),
        });
    }
});

router.post('/reset-password', checkAdmin, async (req, res) => {
    const { email } = req.body;
    const emailLC = email.toLowerCase();
    try {
        const passwordHex = await crypto.randomBytes(12);
        const password = passwordHex.toString('hex');
        const salt = await bcrypt.genSalt(saltRounds);
        const newPasswordHash = await bcrypt.hash(password, salt);
        await db.query('update users set password_hash =$1 where email=$2', [
            newPasswordHash,
            emailLC,
        ]);
        res.status(200).json({
            success: 'Password succesfully changed',
            newPassword: password,
        });
    } catch (e) {
        res.status(500).json({
            error: e.toString(),
        });
    }
});
