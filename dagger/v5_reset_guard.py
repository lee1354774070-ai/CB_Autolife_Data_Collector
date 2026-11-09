"""S3 reset escape: move only head pitch out of an existing head/shoulder overlap.

No new contacts are permitted. All other joints remain at their measured pose
until head feedback reaches the checked waypoint; the usual full-path guard
then owns the rest of the reset.
"""
import numpy as np

HEAD_PAIR = frozenset(('Link_Waist_Yaw_to_Shoulder_Inner_0', 'Link_Neck_Yaw_to_Head_0'))


def collision_depths(kinematics, q):
    kinematics.collision_pairs(q)
    result = {}
    for index, pair in enumerate(kinematics.geom_model.collisionPairs):
        contacts = kinematics.geom_data.collisionResults[index]
        if not contacts.isCollision():
            continue
        count = contacts.numContacts()
        if not 0 < count < 64:
            raise ValueError('collision contact depth unavailable or truncated')
        names = frozenset((kinematics.geom_model.geometryObjects[pair.first].name,
                           kinematics.geom_model.geometryObjects[pair.second].name))
        depth = max(abs(float(contacts.getContact(i).penetration_depth)) for i in range(count))
        if not np.isfinite(depth):
            raise ValueError('nonfinite collision depth')
        result[names] = max(result.get(names, 0.), depth)
    return result


def prepare_reset(controller, measured, goal_groups):
    from openarmx_teleop_vr_306_v4.reset_path import conservative_reset_duration
    k = controller._kinematics
    direct = controller._prepare_direct_quick_reset_path
    if not k.collision_available:
        return direct(measured, goal_groups)
    requests = [(r, r.enable_contact, r.num_max_contacts) for r in k.geom_data.collisionRequests]
    try:
        for r, _, _ in requests:
            r.enable_contact = True; r.num_max_contacts = 64
        start = controller._group_vector(measured).astype(float, copy=True)
        goal = controller._group_vector(goal_groups).astype(float, copy=True)
        initial = collision_depths(k, k.q_from_feedback(measured, clip=False))
        if not initial:
            return direct(measured, goal_groups)
        if set(initial) != {HEAD_PAIR}:
            raise ValueError('起点存在其他碰撞，不能自动抬头避让')
        if not goal[19] < start[19] - .1:
            raise ValueError('任务姿态未提供更高的头部位置，不能自动避让')
        waypoint = start.copy(); waypoint[19] = goal[19]
        last_depth = initial[HEAD_PAIR]
        indices = list(k.joint_q.values())
        # Dense samples in physical angle, independent of the reset clock.
        for pitch in np.linspace(start[19], waypoint[19], max(2, int(np.ceil((start[19]-waypoint[19])/.1))+1)):
            sample = start.copy(); sample[19] = pitch
            groups = controller._kinematics_groups_from_vector(sample)
            q = k.q_from_feedback(groups, clip=False)
            if (np.any(q[indices] < k.lower[indices]-1e-6)
                    or np.any(q[indices] > k.upper[indices]+1e-6)):
                raise ValueError('抬头避让路径超出关节限位')
            contacts = collision_depths(k, q)
            depth = contacts.get(HEAD_PAIR, 0.)
            if set(contacts) - {HEAD_PAIR} or depth > last_depth + 1e-6:
                raise ValueError('抬头避让产生新碰撞或加深原碰撞')
            last_depth = depth
        if last_depth > 0:
            raise ValueError('任务头部姿态仍在碰撞区')
        # The physical completion tolerance must also be collision-free.
        margin = waypoint.copy(); margin[19] += .25
        if collision_depths(k, k.q_from_feedback(controller._kinematics_groups_from_vector(margin), clip=False)):
            raise ValueError('抬头避让终点余量不足')
        error, remainder = direct(controller._kinematics_groups_from_vector(waypoint), goal_groups)
        if error:
            return error, None
        head_duration = conservative_reset_duration(start, waypoint,
            float(controller.get_parameter('quick_reset_max_velocity_deg_sec').value),
            float(controller.get_parameter('quick_reset_neck_max_velocity_deg_sec').value),
            float(controller.get_parameter('quick_reset_minimum_path_duration_sec').value))
        duration = head_duration + remainder['duration_sec']
        return '', dict(start=start, goal=goal, duration_sec=duration,
                        head_waypoint=waypoint, head_split=head_duration/duration)
    except Exception as error:
        return '全身复位抬头避让检查失败：' + str(error), None
    finally:
        for r, enabled, count in requests:
            r.enable_contact = enabled; r.num_max_contacts = count


def head_first_sample(start, waypoint, goal, progress, split):
    from openarmx_teleop_vr_306_v4.reset_path import continuous_direct_reset
    if progress <= split:
        return continuous_direct_reset(start, waypoint, progress/split)
    return continuous_direct_reset(waypoint, goal, (progress-split)/(1-split))


def head_first_progress(start, waypoint, goal, measured, previous, proposed, split, lead):
    from openarmx_teleop_vr_306_v4.reset_path import smoothstep5, feedback_bounded_reset_progress
    if previous < split:
        high = min(proposed, split)/split
        low = previous/split
        delta = waypoint[19]-start[19]
        allowed = (measured[19]-start[19])/delta + 1./abs(delta)
        if smoothstep5(high) > allowed:
            for _ in range(24):
                middle = (low+high)*.5
                if smoothstep5(middle) <= allowed: low = middle
                else: high = middle
            high = low
        return max(previous, high*split)
    if abs(measured[19]-waypoint[19]) > .25:
        return previous  # Arms/waist cannot move before measured head clearance.
    bounded = feedback_bounded_reset_progress(waypoint, goal, measured,
        (previous-split)/(1-split), (proposed-split)/(1-split), lead)
    return split+(1-split)*bounded
