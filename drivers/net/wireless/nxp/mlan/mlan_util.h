/** @file mlan_util.h
 *
 *  @brief This file contains wrappers for linked-list,
 *  spinlock and timer defines.
 *
 *
 *  Copyright 2008-2021, 2025 NXP
 *
 *  This software file (the File) is distributed by NXP
 *  under the terms of the GNU General Public License Version 2, June 1991
 *  (the License).  You may use, redistribute and/or modify the File in
 *  accordance with the terms and conditions of the License, a copy of which
 *  is available by writing to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 *  worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 *  THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 *  ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 *  this warranty disclaimer.
 *
 */

/******************************************************
Change log:
    10/28/2008: initial version
******************************************************/

#ifndef _MLAN_UTIL_H_
#define _MLAN_UTIL_H_

/** Circular doubly linked list */
typedef struct _mlan_linked_list {
	/** Pointer to previous node */
	struct _mlan_linked_list *pprev;
	/** Pointer to next node */
	struct _mlan_linked_list *pnext;
} mlan_linked_list, *pmlan_linked_list;

/** List head */
typedef struct _mlan_list_head {
	/** Pointer to previous node */
	struct _mlan_linked_list *pprev;
	/** Pointer to next node */
	struct _mlan_linked_list *pnext;
	/** Pointer to lock */
	t_void *plock;
} mlan_list_head, *pmlan_list_head;

struct reflective_enum_element {
	int id;
	const char *name;
};

/** MLAN MNULL pointer */
#define MNULL ((void *)0)

/**
 *  @brief This function initializes a list without locking
 *
 *  @param phead		List head
 *
 *  @return			N/A
 */
static INLINE t_void util_init_list(pmlan_linked_list phead)
{
	/* Both next and prev point to self */
	phead->pprev = phead->pnext = (pmlan_linked_list)phead;
}

/**
 *  @brief This function initializes a list head without locking
 *
 *  @param phead		List head
 *
 *  @return			N/A
 */
static INLINE t_void util_list_head_reset(pmlan_list_head phead)
{
	/* Both next and prev point to self */
	phead->pprev = phead->pnext = (pmlan_linked_list)phead;
}

/**
 *  @brief This function initializes a list
 *
 *  @param phead		List head
 *  @param lock_required	A flag for spinlock requirement
 *  @param moal_init_lock	A pointer to init lock handler
 *
 *  @return			N/A
 */
static INLINE t_void util_init_list_head(
	t_void *pmoal_handle, pmlan_list_head phead, t_u8 lock_required,
	mlan_status (*moal_init_lock)(t_void *handle, t_void **pplock))
{
	/* Both next and prev point to self */
	util_init_list((pmlan_linked_list)phead);
	if (lock_required)
		moal_init_lock(pmoal_handle, &phead->plock);
	else
		phead->plock = MNULL;
}

/**
 *  @brief This function frees a list
 *
 *  @param phead		List head
 *  @param moal_free_lock	A pointer to free lock handler
 *
 *  @return			N/A
 */
static INLINE t_void util_free_list_head(
	t_void *pmoal_handle, pmlan_list_head phead,
	mlan_status (*moal_free_lock)(t_void *handle, t_void *plock))
{
	phead->pprev = phead->pnext = MNULL;
	if (phead->plock)
		moal_free_lock(pmoal_handle, phead->plock);
}

/**
 *  @brief This function peeks into a list without lock
 *
 *  @param phead		List head
 *
 *  @return			List node
 */
static INLINE pmlan_linked_list util_peek_list_nl(t_void *pmoal_handle,
						  pmlan_list_head phead)
{
	pmlan_linked_list pnode = MNULL;

	if (phead->pnext != (pmlan_linked_list)phead)
		pnode = phead->pnext;

	return pnode;
}

/**
 *  @brief This function peeks into a list
 *
 *  @param phead		List head
 *  @param moal_spin_lock	A pointer to spin lock handler
 *  @param moal_spin_unlock	A pointer to spin unlock handler
 *
 *  @return			List node
 */
static INLINE pmlan_linked_list
util_peek_list(t_void *pmoal_handle, pmlan_list_head phead,
	       mlan_status (*moal_spin_lock)(t_void *handle, t_void *plock),
	       mlan_status (*moal_spin_unlock)(t_void *handle, t_void *plock))
{
	pmlan_linked_list pnode = MNULL;

	if (moal_spin_lock)
		moal_spin_lock(pmoal_handle, phead->plock);

	pnode = util_peek_list_nl(pmoal_handle, phead);

	if (moal_spin_unlock)
		moal_spin_unlock(pmoal_handle, phead->plock);
	return pnode;
}

/**
 *  @brief This function queues a node at the list tail without taking any lock
 *
 *  @param phead		List head
 *  @param pnode		List node to queue
 *
 *  @return			N/A
 */
static INLINE t_void util_enqueue_list_tail_nl(t_void *pmoal_handle,
					       pmlan_list_head phead,
					       pmlan_linked_list pnode)
{
	pmlan_linked_list pold_last = phead->pprev;
	pnode->pprev = pold_last;
	pnode->pnext = (pmlan_linked_list)phead;

	phead->pprev = pold_last->pnext = pnode;
}

/**
 *  @brief This function queues a node at the list tail
 *
 *  @param phead		List head
 *  @param pnode		List node to queue
 *  @param moal_spin_lock	A pointer to spin lock handler
 *  @param moal_spin_unlock	A pointer to spin unlock handler
 *
 *  @return			N/A
 */
static INLINE t_void util_enqueue_list_tail(
	t_void *pmoal_handle, pmlan_list_head phead, pmlan_linked_list pnode,
	mlan_status (*moal_spin_lock)(t_void *handle, t_void *plock),
	mlan_status (*moal_spin_unlock)(t_void *handle, t_void *plock))
{
	if (moal_spin_lock)
		moal_spin_lock(pmoal_handle, phead->plock);

	util_enqueue_list_tail_nl(pmoal_handle, phead, pnode);

	if (moal_spin_unlock)
		moal_spin_unlock(pmoal_handle, phead->plock);
}

/**
 *  @brief This function adds a node at the list head
 *
 *  @param phead		List head
 *  @param pnode		List node to add
 *  @param moal_spin_lock	A pointer to spin lock handler
 *  @param moal_spin_unlock	A pointer to spin unlock handler
 *
 *  @return			N/A
 */
static INLINE t_void util_enqueue_list_head(
	t_void *pmoal_handle, pmlan_list_head phead, pmlan_linked_list pnode,
	mlan_status (*moal_spin_lock)(t_void *handle, t_void *plock),
	mlan_status (*moal_spin_unlock)(t_void *handle, t_void *plock))
{
	pmlan_linked_list pold_first;

	if (moal_spin_lock)
		moal_spin_lock(pmoal_handle, phead->plock);
	pold_first = phead->pnext;
	pnode->pprev = (pmlan_linked_list)phead;
	pnode->pnext = pold_first;

	phead->pnext = pold_first->pprev = pnode;
	if (moal_spin_unlock)
		moal_spin_unlock(pmoal_handle, phead->plock);
}

/**
 *  @brief This function checks if the node points to itself
 *
 *  @param pnode		List node to check
 *
 *  @return			MTRUE if node points to itself only
 */
static INLINE t_bool util_is_node_itself(pmlan_linked_list pnode)
{
	return pnode->pprev == pnode && pnode->pnext == pnode;
}

/**
 *  @brief This function checks if the node in some list
 *
 *  @param pnode		List node to check
 *
 *  @return			MTRUE if node is enqueued into some list
 */
static INLINE t_bool util_is_node_in_list(pmlan_linked_list pnode)
{
	return pnode->pprev && pnode->pnext && !util_is_node_itself(pnode);
}

/**
 *  @brief This function checks if the pnode is valid node of list
 *
 *  @param phead		List`s head
 *  @param pnode		List node to check
 *
 *  @return			MTRUE if node is enqueued into some list
 */
static INLINE t_bool util_is_list_node(pmlan_list_head phead,
				       pmlan_linked_list pnode)
{
	return pnode && (pmlan_linked_list)phead != pnode;
}

/**
 *  @brief This function removes a node from the list if the node was in the
 * list
 *
 *  @param pnode		List node to remove
 *
 *  @return			N/A
 */
static INLINE t_void util_unlink_list_safe_nl(t_void *pmoal_handle,
					      pmlan_linked_list pnode)
{
	if (util_is_node_in_list(pnode)) {
		pmlan_linked_list pmy_prev;
		pmlan_linked_list pmy_next;

		pmy_prev = pnode->pprev;
		pmy_next = pnode->pnext;
		pmy_next->pprev = pmy_prev;
		pmy_prev->pnext = pmy_next;

		pnode->pnext = pnode->pprev = MNULL;
	}
}

/**
 *  @brief This function removes a node from the list
 *
 *  @param pnode		List node to remove
 *
 *  @return			N/A
 */
static INLINE t_void util_unlink_list_nl(t_void *pmoal_handle,
					 pmlan_linked_list pnode)
{
	pmlan_linked_list pmy_prev;
	pmlan_linked_list pmy_next;

	pmy_prev = pnode->pprev;
	pmy_next = pnode->pnext;
	pmy_next->pprev = pmy_prev;
	pmy_prev->pnext = pmy_next;

	pnode->pnext = pnode->pprev = MNULL;
}

/**
 *  @brief This function removes a node from the list
 *
 *  @param phead		List head
 *  @param pnode		List node to remove
 *  @param moal_spin_lock	A pointer to spin lock handler
 *  @param moal_spin_unlock	A pointer to spin unlock handler
 *
 *  @return			N/A
 */
static INLINE t_void util_unlink_list(
	t_void *pmoal_handle, pmlan_list_head phead, pmlan_linked_list pnode,
	mlan_status (*moal_spin_lock)(t_void *handle, t_void *plock),
	mlan_status (*moal_spin_unlock)(t_void *handle, t_void *plock))
{
	if (moal_spin_lock)
		moal_spin_lock(pmoal_handle, phead->plock);

	util_unlink_list_nl(pmoal_handle, pnode);

	if (moal_spin_unlock)
		moal_spin_unlock(pmoal_handle, phead->plock);
}

/**
 *  @brief This function dequeues a node from the list
 *
 *  @param phead		List head
 *  @param moal_spin_lock	A pointer to spin lock handler
 *  @param moal_spin_unlock	A pointer to spin unlock handler
 *
 *  @return			List node
 */
static INLINE pmlan_linked_list util_dequeue_list(
	t_void *pmoal_handle, pmlan_list_head phead,
	mlan_status (*moal_spin_lock)(t_void *handle, t_void *plock),
	mlan_status (*moal_spin_unlock)(t_void *handle, t_void *plock))
{
	pmlan_linked_list pnode;

	if (moal_spin_lock)
		moal_spin_lock(pmoal_handle, phead->plock);
	pnode = phead->pnext;
	if (pnode && (pnode != (pmlan_linked_list)phead))
		util_unlink_list(pmoal_handle, phead, pnode, MNULL, MNULL);
	else
		pnode = MNULL;
	if (moal_spin_unlock)
		moal_spin_unlock(pmoal_handle, phead->plock);
	return pnode;
}

/** Access controlled scalar variable */
typedef struct _mlan_scalar {
	/** Value */
	t_s32 value;
	/** Pointer to lock */
	t_void *plock;
	/** Control flags */
	t_u32 flags;
} mlan_scalar, *pmlan_scalar;

/** Flag to scalar lock acquired */
#define MLAN_SCALAR_FLAG_UNIQUE_LOCK MBIT(16)

/** scalar conditional value list */
typedef enum _MLAN_SCALAR_CONDITIONAL {
	MLAN_SCALAR_COND_EQUAL,
	MLAN_SCALAR_COND_NOT_EQUAL,
	MLAN_SCALAR_COND_GREATER_THAN,
	MLAN_SCALAR_COND_GREATER_OR_EQUAL,
	MLAN_SCALAR_COND_LESS_THAN,
	MLAN_SCALAR_COND_LESS_OR_EQUAL
} MLAN_SCALAR_CONDITIONAL;

/**
 *  @brief This function initializes a scalar
 *
 *  @param pscalar			Pointer to scalar
 *  @param val				Initial scalar value
 *  @param plock_to_use		A new lock is created if NULL, else lock to use
 *  @param moal_init_lock	A pointer to init lock handler
 *
 *  @return					N/A
 */
static INLINE t_void
util_scalar_init(t_void *pmoal_handle, pmlan_scalar pscalar, t_s32 val,
		 t_void *plock_to_use,
		 mlan_status (*moal_init_lock)(t_void *handle, t_void **pplock))
{
	pscalar->value = val;
	pscalar->flags = 0;
	if (plock_to_use) {
		pscalar->flags &= ~MLAN_SCALAR_FLAG_UNIQUE_LOCK;
		pscalar->plock = plock_to_use;
	} else {
		pscalar->flags |= MLAN_SCALAR_FLAG_UNIQUE_LOCK;
		moal_init_lock(pmoal_handle, &pscalar->plock);
	}
}

/**
 *  @brief This function frees a scalar
 *
 *  @param pscalar			Pointer to scalar
 *  @param moal_free_lock	A pointer to free lock handler
 *
 *  @return			N/A
 */
static INLINE t_void
util_scalar_free(t_void *pmoal_handle, pmlan_scalar pscalar,
		 mlan_status (*moal_free_lock)(t_void *handle, t_void *plock))
{
	if (pscalar->flags & MLAN_SCALAR_FLAG_UNIQUE_LOCK)
		moal_free_lock(pmoal_handle, pscalar->plock);
}

/**
 *  @brief This function reads value from scalar
 *
 *  @param pscalar			Pointer to scalar
 *  @param moal_spin_lock	A pointer to spin lock handler
 *  @param moal_spin_unlock	A pointer to spin unlock handler
 *
 *  @return					Stored value
 */
static INLINE t_s32
util_scalar_read(t_void *pmoal_handle, pmlan_scalar pscalar,
		 mlan_status (*moal_spin_lock)(t_void *handle, t_void *plock),
		 mlan_status (*moal_spin_unlock)(t_void *handle, t_void *plock))
{
	t_s32 val;

	if (moal_spin_lock)
		moal_spin_lock(pmoal_handle, pscalar->plock);
	val = pscalar->value;
	if (moal_spin_unlock)
		moal_spin_unlock(pmoal_handle, pscalar->plock);

	return val;
}

/**
 *  @brief This function writes value to scalar
 *
 *  @param pscalar			Pointer to scalar
 *  @param val				Value to write
 *  @param moal_spin_lock	A pointer to spin lock handler
 *  @param moal_spin_unlock	A pointer to spin unlock handler
 *
 *  @return					N/A
 */
static INLINE t_void util_scalar_write(
	t_void *pmoal_handle, pmlan_scalar pscalar, t_s32 val,
	mlan_status (*moal_spin_lock)(t_void *handle, t_void *plock),
	mlan_status (*moal_spin_unlock)(t_void *handle, t_void *plock))
{
	if (moal_spin_lock)
		moal_spin_lock(pmoal_handle, pscalar->plock);
	pscalar->value = val;
	if (moal_spin_unlock)
		moal_spin_unlock(pmoal_handle, pscalar->plock);
}

/**
 *  @brief This function increments the value in scalar
 *
 *  @param pscalar			Pointer to scalar
 *  @param moal_spin_lock	A pointer to spin lock handler
 *  @param moal_spin_unlock	A pointer to spin unlock handler
 *
 *  @return					N/A
 */
static INLINE t_void util_scalar_increment(
	t_void *pmoal_handle, pmlan_scalar pscalar,
	mlan_status (*moal_spin_lock)(t_void *handle, t_void *plock),
	mlan_status (*moal_spin_unlock)(t_void *handle, t_void *plock))
{
	if (moal_spin_lock)
		moal_spin_lock(pmoal_handle, pscalar->plock);
	pscalar->value++;
	if (moal_spin_unlock)
		moal_spin_unlock(pmoal_handle, pscalar->plock);
}

/**
 *  @brief This function decrements the value in scalar
 *
 *  @param pscalar			Pointer to scalar
 *  @param moal_spin_lock	A pointer to spin lock handler
 *  @param moal_spin_unlock	A pointer to spin unlock handler
 *
 *  @return					N/A
 */
static INLINE t_void util_scalar_decrement(
	t_void *pmoal_handle, pmlan_scalar pscalar,
	mlan_status (*moal_spin_lock)(t_void *handle, t_void *plock),
	mlan_status (*moal_spin_unlock)(t_void *handle, t_void *plock))
{
	if (moal_spin_lock)
		moal_spin_lock(pmoal_handle, pscalar->plock);
	pscalar->value--;
	if (moal_spin_unlock)
		moal_spin_unlock(pmoal_handle, pscalar->plock);
}

/**
 *  @brief This function adds an offset to the value in scalar,
 *         and returns the new value
 *
 *  @param pscalar			Pointer to scalar
 *  @param offset			Offset value (can be negative)
 *  @param moal_spin_lock	A pointer to spin lock handler
 *  @param moal_spin_unlock	A pointer to spin unlock handler
 *
 *  @return			Value after offset or 0 if (scalar_value + offset)
 * overflows
 */
#define INT_MAX 2147483647
static INLINE t_s32 util_scalar_offset(
	t_void *pmoal_handle, pmlan_scalar pscalar, t_s32 offset,
	mlan_status (*moal_spin_lock)(t_void *handle, t_void *plock),
	mlan_status (*moal_spin_unlock)(t_void *handle, t_void *plock))
{
	t_s64 newval;

	if (moal_spin_lock)
		moal_spin_lock(pmoal_handle, pscalar->plock);
	newval = (pscalar->value += offset);
	if (newval > SINT32_MAX || newval < SINT32_MIN)
		newval = 0;
	if (moal_spin_unlock)
		moal_spin_unlock(pmoal_handle, pscalar->plock);

	return (t_s32)newval;
}

/**
 *  @brief This function writes the value to the scalar
 *         if existing value compared with other value is true.
 *
 *  @param pscalar          Pointer to scalar
 *  @param condition        Condition to check
 *  @param val_compare      Value to compare against current value
 *                          ((A X B), where B = val_compare)
 *  @param val_to_set       Value to set if comparison is true
 *  @param moal_spin_lock   A pointer to spin lock handler
 *  @param moal_spin_unlock A pointer to spin unlock handler
 *
 *  @return                 Comparison result (MTRUE or MFALSE)
 */
static INLINE t_u8 util_scalar_conditional_write(
	t_void *pmoal_handle, pmlan_scalar pscalar,
	MLAN_SCALAR_CONDITIONAL condition, t_s32 val_compare, t_s32 val_to_set,
	mlan_status (*moal_spin_lock)(t_void *handle, t_void *plock),
	mlan_status (*moal_spin_unlock)(t_void *handle, t_void *plock))
{
	t_u8 update;
	if (moal_spin_lock)
		moal_spin_lock(pmoal_handle, pscalar->plock);

	switch (condition) {
	case MLAN_SCALAR_COND_EQUAL:
		update = (pscalar->value == val_compare);
		break;
	case MLAN_SCALAR_COND_NOT_EQUAL:
		update = (pscalar->value != val_compare);
		break;
	case MLAN_SCALAR_COND_GREATER_THAN:
		update = (pscalar->value > val_compare);
		break;
	case MLAN_SCALAR_COND_GREATER_OR_EQUAL:
		update = (pscalar->value >= val_compare);
		break;
	case MLAN_SCALAR_COND_LESS_THAN:
		update = (pscalar->value < val_compare);
		break;
	case MLAN_SCALAR_COND_LESS_OR_EQUAL:
		update = (pscalar->value <= val_compare);
		break;
	default:
		update = MFALSE;
		break;
	}
	if (update)
		pscalar->value = val_to_set;

	if (moal_spin_unlock)
		moal_spin_unlock(pmoal_handle, pscalar->plock);
	return (update) ? MTRUE : MFALSE;
}

/**
 *  @brief This function counts the bits of unsigned int number
 *
 *  @param num  number
 *  @return     number of bits
 */
static INLINE t_u32 bitcount(t_u32 num)
{
	t_u32 count = 0;
	static t_u32 nibblebits[] = {0, 1, 1, 2, 1, 2, 2, 3,
				     1, 2, 2, 3, 2, 3, 3, 4};
	for (; num != 0; num >>= 4)
		count += nibblebits[num & 0x0f];
	return count;
}

static INLINE const char *
reflective_enum_lookup_name(const struct reflective_enum_element *elements,
			    int id)
{
	const struct reflective_enum_element *elem = elements;

	while (elem->name && elem->id != id) {
		elem++;
	}

	return elem->name;
}

#define util_offsetof(struct_type, member_name)                                \
	((t_ptr) & ((struct_type *)0)->member_name)

#define util_container_of(ptr, struct_type, member_name)                       \
	((struct_type *)((t_u8 *)(ptr)-util_offsetof(struct_type, member_name)))

/**
 *  @brief This function checks if t1 timestamp is before t2 timestamp
 *
 *  @param t1   t1 timestamp
 *  @param t2   t2 timestamp
 *
 *  @return     MTRUE if t1 is before t2
 */
static INLINE t_bool util_is_time_before(t_u64 t1, t_u64 t2)
{
	t_s64 delta = t2 - t1;

	// coverity[integer_overflow:SUPPRESS]
	return delta > 0;
}

#endif /* !_MLAN_UTIL_H_ */
