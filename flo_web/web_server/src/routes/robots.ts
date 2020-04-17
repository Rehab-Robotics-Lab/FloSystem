import Router from 'express-promise-router';
import * as db from '../db';
// create a new express-promise-router
// this has the same API as the normal express router except
// it allows you to use async functions as route handlers
const router = Router();
// export our router to be mounted by the parent application
export default router;

const saltRounds = 10;

router.get('/:id', async (req, res) => {
    const { id } = req.params;
    const { rows } = await db.query('SELECT * FROM robots WHERE id = $1', [id]);
    res.send(rows[0]);
});

router.get('/', async (req, res) => {
    const { rows } = await db.query('select * from robots', []);
    res.send(rows);
});
