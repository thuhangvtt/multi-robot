import json
import os
from pathlib import Path


class TaskRegistry:
    def __init__(self, path, pallet_ids, goal_ids):
        self.path = Path(path) if path is not None else None
        self.pallet_ids = list(pallet_ids)
        self.goal_ids = list(goal_ids)
        if (len(set(self.pallet_ids)) != len(self.pallet_ids)
                or len(set(self.goal_ids)) != len(self.goal_ids)
                or set(self.pallet_ids) & set(self.goal_ids)):
            raise ValueError('Pallet and goal IDs must be unique and disjoint')
        self.tasks = {}
        if self.path is not None and self.path.exists():
            saved = json.loads(self.path.read_text(encoding='utf-8'))
            if saved.get('version') == 2:
                self.tasks = saved['tasks']
            else:
                self.tasks = {k: t for k, t in saved.items() if t['state'] != 'PENDING'}
        for task in self.tasks.values():
            if task['pallet_id'] not in self.pallet_ids or (
                    task['state'] != 'PENDING' and task['goal_id'] not in self.goal_ids):
                raise ValueError('Saved task IDs differ from configured marker classes')
            if task['state'] == 'PENDING':
                task['goal_id'] = None
        if any(t['state'] not in ('PENDING', 'DELIVERED') for t in self.tasks.values()):
            raise RuntimeError('Unverified previous mission: inspect payloads and reconcile task ledger before restarting')

    def save(self):
        if self.path is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps({'version': 2, 'pairing': 'parrot_visible_order',
                                        'tasks': self.tasks}, indent=2), encoding='utf-8')
        os.replace(temporary, self.path)

    def pending(self, markers):
        
        discovered = False
        for fid in self.pallet_ids:
            if fid in markers and str(fid) not in self.tasks:
                self.tasks[str(fid)] = dict(pallet_id=fid, goal_id=None, state='PENDING', robot_id=None)
                discovered = True
        for task in self.tasks.values():
            if task['state'] == 'PENDING':
                task['goal_id'] = None
        if discovered:
            self.save()
        used_goals = {t['goal_id'] for t in self.tasks.values() if t['state'] != 'PENDING'}
        pallets = [self.tasks[str(fid)] for fid in self.pallet_ids
                   if fid in markers and self.tasks[str(fid)]['state'] == 'PENDING']
        goals = [fid for fid in self.goal_ids if fid in markers and fid not in used_goals]
        proposals = []
        for task, goal in zip(pallets, goals):
            task['goal_id'] = goal
            proposals.append(task)
        return proposals

    def all_delivered(self):
        return bool(self.tasks) and all(t['state'] == 'DELIVERED' for t in self.tasks.values())

    def assign(self, tasks, assignments, hw_ids):
        if len(assignments) != len(hw_ids):
            raise ValueError('Assignments must match robot IDs')
        selected = [index for index in assignments if index >= 0]
        if len(set(selected)) != len(selected) or any(index >= len(tasks) for index in selected):
            raise ValueError('Invalid or duplicate task assignment')
        for index in selected:
            task = tasks[index]
            if (self.tasks.get(str(task['pallet_id'])) is not task
                    or task['state'] != 'PENDING' or task['goal_id'] is None):
                raise ValueError('Only a pending visible pallet-goal pair can be assigned')
        active = []
        for index, hw in zip(assignments, hw_ids):
            if index >= 0:
                task = tasks[index]
                task.update(state='ASSIGNED', robot_id=hw)
                active.append(task)
        self.save()
        return active

    def awaiting_verification(self, tasks):
        for task in tasks:
            task['state'] = 'AWAITING_VERIFICATION'
        self.save()

    def confirm(self, tasks):
        for task in tasks:
            if task['state'] != 'AWAITING_VERIFICATION':
                raise ValueError('Cannot confirm an unfinished task')
            task['state'] = 'DELIVERED'
        self.save()

    def complete_round(self, tasks):
        """Controller finished this round; no claim of physical verification."""
        for task in tasks:
            task['state'] = 'DELIVERED'
        self.save()
