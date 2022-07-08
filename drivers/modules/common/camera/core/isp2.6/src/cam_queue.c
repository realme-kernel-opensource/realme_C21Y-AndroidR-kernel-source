/*
 * Copyright (C) 2020-2021 UNISOC Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/slab.h>

#include "cam_types.h"
#include "cam_buf.h"
#include "cam_queue.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "cam_queue: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

int cam_queue_enqueue(struct camera_queue *q, struct list_head *list)
{
	int ret = 0;
	unsigned long flags;

	if (q == NULL || list == NULL) {
		pr_err("fail to get valid param %p %p\n", q, list);
		return -EINVAL;
	}

	spin_lock_irqsave(&q->lock, flags);
	if (q->state == CAM_Q_CLEAR) {
		pr_warn("q is clear\n");
		ret = -EPERM;
		goto unlock;
	}

	if (q->cnt >= q->max) {
		pr_warn("q full, cnt %d, max %d\n", q->cnt, q->max);
		ret = -EPERM;
		goto unlock;
	}
	q->cnt++;
	list_add_tail(list, &q->head);

unlock:
	spin_unlock_irqrestore(&q->lock, flags);

	return ret;
}

/* dequeue frame from tail of queue */
struct camera_frame *cam_queue_dequeue_tail(struct camera_queue *q)
{
	int fatal_err;
	unsigned long flags;
	struct camera_frame *pframe = NULL;

	if (q == NULL) {
		pr_err("fail to get valid param\n");
		return NULL;
	}

	spin_lock_irqsave(&q->lock, flags);
	if (q->state == CAM_Q_CLEAR) {
		pr_warn("q is clear\n");
		goto unlock;
	}

	if (list_empty(&q->head) || q->cnt == 0) {
		pr_debug("queue empty %d, %d\n",
			list_empty(&q->head), q->cnt);
		fatal_err = (list_empty(&q->head) ^ (q->cnt == 0));
		if (fatal_err)
			pr_debug("error:  empty %d, cnt %d\n",
				list_empty(&q->head), q->cnt);
		goto unlock;
	}

	pframe = list_last_entry(&q->head, struct camera_frame, list);
	list_del(&pframe->list);
	q->cnt--;
unlock:
	spin_unlock_irqrestore(&q->lock, flags);

	return pframe;
}

struct camera_frame *cam_queue_dequeue_if(struct camera_queue *q,
	bool (*filter)(struct camera_frame *, void *), void *data)
{
	int fatal_err;
	unsigned long flags;
	struct camera_frame *pframe = NULL;

	if (q == NULL || !filter) {
		pr_err("fail to get valid param %p %p\n", q, filter);
		return NULL;
	}

	spin_lock_irqsave(&q->lock, flags);
	if (q->state == CAM_Q_CLEAR) {
		pr_warn("q is clear\n");
		goto unlock;
	}

	if (list_empty(&q->head) || q->cnt == 0) {
		pr_debug("queue empty %d, %d\n",
			list_empty(&q->head), q->cnt);
		fatal_err = (list_empty(&q->head) ^ (q->cnt == 0));
		if (fatal_err)
			pr_err("fail to get list node, empty %d, cnt %d\n",
				list_empty(&q->head), q->cnt);
		goto unlock;
	}

	pframe = list_first_entry(&q->head, struct camera_frame, list);

	if (!filter(pframe, data)) {
		pframe = NULL;
		goto unlock;
	}

	list_del(&pframe->list);
	q->cnt--;

unlock:
	spin_unlock_irqrestore(&q->lock, flags);

	return pframe;
}

int cam_queue_init(struct camera_queue *q,
	uint32_t max, void (*cb_func)(void *))
{
	int ret = 0;

	if (q == NULL) {
		pr_err("fail to get valid param\n");
		return -EINVAL;
	}
	q->cnt = 0;
	q->max = max;
	q->state = CAM_Q_INIT;
	q->destroy = cb_func;
	spin_lock_init(&q->lock);
	INIT_LIST_HEAD(&q->head);

	return ret;
}

/* get the number of how many buf in the queue */
uint32_t cam_queue_cnt_get(struct camera_queue *q)
{
	unsigned long flags;
	uint32_t tmp;

	spin_lock_irqsave(&q->lock, flags);
	tmp = q->cnt;
	spin_unlock_irqrestore(&q->lock, flags);

	return tmp;
}

/* Get the same time two frame from two queue
 * Input: q0, q1, two queue
 *        *pf0, *pf1, return the two frame
 *        t, no use now
 */
int cam_queue_same_frame_get(struct camera_queue *q0,
	struct camera_queue *q1,struct camera_frame **pf0,
	struct camera_frame **pf1, int64_t t)
{
	int ret = 0;
	unsigned long flags0, flags1;
	struct camera_frame *tmpf0, *tmpf1;
	int64_t t0, t1, mint;

	spin_lock_irqsave(&q0->lock, flags0);
	spin_lock_irqsave(&q1->lock, flags1);
	if (list_empty(&q0->head) || list_empty(&q1->head)) {
		pr_err("fail to get list node\n");
		ret = -EFAULT;
		goto _EXT;
	}
	/* set mint a large value */
	mint = (((uint64_t)1 << 63) - 1);
	/* get least delta time two frame */
	list_for_each_entry(tmpf0, &q0->head, list) {
		t0 = tmpf0->sensor_time.tv_sec * 1000000ll;
		t0 += tmpf0->sensor_time.tv_usec;
		list_for_each_entry(tmpf1, &q1->head, list) {
			t1 = tmpf1->sensor_time.tv_sec * 1000000ll;
			t1 += tmpf1->sensor_time.tv_usec;
			t1 -= t0;
			if (t1 < 0)
				t1 = -t1;
			if (t1 < mint) {
				*pf0 = tmpf0;
				*pf1 = tmpf1;
				mint = t1;
			}
		}
		/* loop 9times --> 3times, 400us */
		if (mint < 400)
			break;
	}
	pr_info("mint:%lld\n", mint);
	if (mint > 50 * 1000) { /* delta > 50ms fail */
		ret = -EFAULT;
		goto _EXT;
	}
	list_del(&((*pf0)->list));
	q0->cnt--;
	list_del(&((*pf1)->list));
	q1->cnt--;

	ret = 0;
_EXT:
	spin_unlock_irqrestore(&q0->lock, flags0);
	spin_unlock_irqrestore(&q1->lock, flags1);

	return ret;
}

/* in irq handler, may return NULL if alloc failed
 * else: will always retry alloc and return valid frame
 */
struct camera_frame *cam_queue_empty_frame_get(void)
{
	int ret = 0;
	uint32_t i;
	struct camera_frame *pframe = NULL;

	pr_debug("Enter.\n");
	do {
		pframe = cam_queue_dequeue(g_empty_frm_q, struct camera_frame, list);
		if (pframe == NULL) {
			if (in_interrupt()) {
				/* fast alloc and return for irq handler */
				pframe = kzalloc(sizeof(*pframe), GFP_ATOMIC);
				if (pframe)
					atomic_inc(&g_mem_dbg->empty_frm_cnt);
				else
					pr_err("fail to alloc memory\n");
				return pframe;
			}

			for (i = 0; i < CAM_EMP_Q_LEN_INC; i++) {
				pframe = kzalloc(sizeof(*pframe), GFP_KERNEL);
				if (pframe == NULL) {
					pr_err("fail to alloc memory, retry\n");
					continue;
				}
				atomic_inc(&g_mem_dbg->empty_frm_cnt);
				pr_debug("alloc frame %p\n", pframe);
				ret = cam_queue_enqueue(g_empty_frm_q, &pframe->list);
				if (ret) {
					/* q full, return pframe directly here */
					break;
				}
				pframe = NULL;
			}
			pr_info("alloc %d empty frames, cnt %d\n",
				i, atomic_read(&g_mem_dbg->empty_frm_cnt));
		}
	} while (pframe == NULL);

	pr_debug("Done. get frame %p\n", pframe);
	return pframe;
}

int cam_queue_empty_frame_put(struct camera_frame *pframe)
{
	int ret = 0;

	if (pframe == NULL) {
		pr_err("fail to get valid param\n");
		return -EINVAL;
	}
	pr_debug("put frame %p\n", pframe);

	memset(pframe, 0, sizeof(struct camera_frame));
	ret = cam_queue_enqueue(g_empty_frm_q, &pframe->list);
	if (ret) {
		pr_info("queue should be enlarged\n");
		atomic_dec(&g_mem_dbg->empty_frm_cnt);
		kfree(pframe);
	}
	return 0;
}

void cam_queue_empty_frame_free(void *param)
{
	struct camera_frame *pframe;

	pframe = (struct camera_frame *)param;
	atomic_dec(&g_mem_dbg->empty_frm_cnt);
	pr_debug("free frame %p, cnt %d\n", pframe,
		atomic_read(&g_mem_dbg->empty_frm_cnt));
	kfree(pframe);
}

struct isp_stream_ctrl *cam_queue_empty_state_get(void)
{
	int ret = 0;
	uint32_t i;
	struct camera_queue *q = g_empty_state_q;
	struct isp_stream_ctrl *stream_state = NULL;

	pr_debug("Enter.\n");
	do {
		stream_state = cam_queue_dequeue(q, struct isp_stream_ctrl, list);
		if (stream_state == NULL) {
			if (in_interrupt()) {
				/* fast alloc and return for irq handler */
				stream_state = kzalloc(sizeof(*stream_state), GFP_ATOMIC);
				if (stream_state)
					atomic_inc(&g_mem_dbg->empty_state_cnt);
				else
					pr_err("fail to alloc memory\n");
				return stream_state;
			}

			for (i = 0; i < CAM_EMP_Q_LEN_INC; i++) {
				stream_state = kzalloc(sizeof(*stream_state), GFP_KERNEL);
				if (stream_state == NULL) {
					pr_err("fail to alloc memory, retry\n");
					continue;
				}
				atomic_inc(&g_mem_dbg->empty_state_cnt);
				pr_debug("alloc frame %p\n", stream_state);
				ret = cam_queue_enqueue(q, &stream_state->list);
				if (ret) {
					/* q full, return pframe directly here */
					break;
				}
				stream_state = NULL;
			}
			pr_debug("alloc %d empty states, cnt %d\n",
				i, atomic_read(&g_mem_dbg->empty_state_cnt));
		}
	} while (stream_state == NULL);

	pr_debug("Done. get state %p\n", stream_state);
	return stream_state;
}

void cam_queue_empty_state_put(void *param)
{
	int ret = 0;
	struct isp_stream_ctrl *stream_state = NULL;

	if (param == NULL) {
		pr_err("fail to get valid param\n");
		return;
	}
	stream_state = (struct isp_stream_ctrl *)param;
	pr_debug("put state %p\n", stream_state);

	memset(stream_state, 0, sizeof(struct isp_stream_ctrl));
	ret = cam_queue_enqueue(g_empty_state_q, &stream_state->list);
	if (ret) {
		pr_debug("queue should be enlarged\n");
		atomic_dec(&g_mem_dbg->empty_state_cnt);
		kfree(stream_state);
	}
}

void cam_queue_empty_state_free(void *param)
{
	struct isp_stream_ctrl *stream_state = NULL;

	stream_state = (struct isp_stream_ctrl *)param;
	atomic_dec(&g_mem_dbg->empty_state_cnt);
	pr_debug("free state %p, cnt %d\n", stream_state,
		atomic_read(&g_mem_dbg->empty_state_cnt));
	kfree(stream_state);
}

