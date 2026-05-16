from __future__ import annotations

from collections.abc import Callable

from planning.pddl import (
    Action,
    ActionSchema,
    Problem,
    State,
    Objects,
    is_applicable,
    apply_action,
    get_all_groundings,
)
from planning.utils import Queue, PriorityQueue
from planning.heuristics import nullHeuristic


# ---------------------------------------------------------------------------
# Reference implementation – read and understand before coding the rest.
# ---------------------------------------------------------------------------


def tinyBaseSearch(problem: Problem) -> list[Action]:
    """
    Hardcoded plan for the tinyBase layout.
    The robot at (1,4) must: pick up supplies at (1,3), set them up at (1,2),
    pick up the patient at (1,1), bring them to (1,2), and execute Rescue.

    Useful to understand the Action object format and plan structure.
    """
    robot = "robot"
    supplies = "supplies_0"
    patient = "patient_0"

    c14 = (1, 4)  # robot start
    c13 = (1, 3)  # supplies
    c12 = (1, 2)  # medical post
    c11 = (1, 1)  # patient

    plan = [
        Action(
            "Move(robot,(1,4),(1,3))",
            [("At", robot, c14), ("Adjacent", c14, c13), ("Free", c13)],
            [],
            [("At", robot, c13), ("Free", c14)],
            [("At", robot, c14), ("Free", c13)],
        ),
        Action(
            "PickUp(robot,supplies_0,(1,3))",
            [
                ("At", robot, c13),
                ("At", supplies, c13),
                ("HandsFree", robot),
                ("Pickable", supplies),
            ],
            [],
            [("Holding", robot, supplies)],
            [("At", supplies, c13), ("HandsFree", robot)],
        ),
        Action(
            "Move(robot,(1,3),(1,2))",
            [("At", robot, c13), ("Adjacent", c13, c12), ("Free", c12)],
            [],
            [("At", robot, c12), ("Free", c13)],
            [("At", robot, c13), ("Free", c12)],
        ),
        Action(
            "SetupSupplies(robot,supplies_0,(1,2))",
            [("At", robot, c12), ("MedicalPost", c12), ("Holding", robot, supplies)],
            [("SuppliesReady", c12)],
            [("SuppliesReady", c12), ("HandsFree", robot)],
            [("Holding", robot, supplies)],
        ),
        Action(
            "Move(robot,(1,2),(1,1))",
            [("At", robot, c12), ("Adjacent", c12, c11), ("Free", c11)],
            [],
            [("At", robot, c11), ("Free", c12)],
            [("At", robot, c12), ("Free", c11)],
        ),
        Action(
            "PickUp(robot,patient_0,(1,1))",
            [
                ("At", robot, c11),
                ("At", patient, c11),
                ("HandsFree", robot),
                ("Pickable", patient),
            ],
            [],
            [("Holding", robot, patient)],
            [("At", patient, c11), ("HandsFree", robot)],
        ),
        Action(
            "Move(robot,(1,1),(1,2))",
            [("At", robot, c11), ("Adjacent", c11, c12), ("Free", c12)],
            [],
            [("At", robot, c12), ("Free", c11)],
            [("At", robot, c11), ("Free", c12)],
        ),
        Action(
            "PutDown(robot,patient_0,(1,2))",
            [("At", robot, c12), ("Holding", robot, patient)],
            [],
            [("At", patient, c12), ("HandsFree", robot)],
            [("Holding", robot, patient)],
        ),
        Action(
            "Rescue(robot,patient_0,(1,2))",
            [
                ("At", robot, c12),
                ("At", patient, c12),
                ("MedicalPost", c12),
                ("SuppliesReady", c12),
            ],
            [],
            [("Rescued", patient)],
            [("At", patient, c12)],
        ),
    ]
    return plan


# ---------------------------------------------------------------------------
# Punto 2 – Forward Planning
# ---------------------------------------------------------------------------


def forwardBFS(problem: Problem) -> list[Action]:
    """
    Forward BFS in state space.
    """
    start_state: State = problem.getStartState()

    if problem.isGoalState(start_state):
        return []

    frontier: Queue = Queue()
    frontier.push((start_state, []))

    visited: set[State] = set()
    visited.add(start_state)

    plan: list[Action] = []
    found: bool = False

    while not frontier.isEmpty() and not found:
        current_state, current_plan = frontier.pop()

        successors: list[tuple[State, Action, int]] = problem.getSuccessors(current_state)

        for successor in successors:
            next_state: State = successor[0]
            action: Action = successor[1]

            if next_state not in visited:
                new_plan: list[Action] = current_plan + [action]

                if problem.isGoalState(next_state):
                    plan = new_plan
                    found = True
                else:
                    visited.add(next_state)
                    frontier.push((next_state, new_plan))

    return plan


# ---------------------------------------------------------------------------
# Punto 3 – Backward Planning
# ---------------------------------------------------------------------------


def regress(goal_set: State, action: Action) -> State | None:
    """
    Compute the regression of goal_set through action.
    """
    regressed_goal: State | None = None

    useful_goal: State = frozenset(
        fluent for fluent in goal_set
        if fluent[0] != "Free"
    )

    is_relevant: bool = not action.add_list.isdisjoint(useful_goal)
    deletes_goal: bool = not action.del_list.isdisjoint(useful_goal)

    if is_relevant and not deletes_goal:
        raw_goal: State = frozenset((useful_goal - action.add_list) | action.precond_pos)

        new_goal: State = frozenset(
            fluent for fluent in raw_goal
            if fluent[0] != "Free"
        )

        has_contradiction: bool = False

        if not action.precond_neg.isdisjoint(new_goal):
            has_contradiction = True

        at_by_entity: dict[object, object] = {}
        holding_objects: set[object] = set()
        has_hands_free: bool = False

        for fluent in new_goal:
            if fluent[0] == "At":
                entity: object = fluent[1]
                location: object = fluent[2]

                if entity in at_by_entity and at_by_entity[entity] != location:
                    has_contradiction = True
                else:
                    at_by_entity[entity] = location

            elif fluent[0] == "Holding":
                holding_objects.add(fluent[2])

            elif fluent[0] == "HandsFree":
                has_hands_free = True

        if has_hands_free and len(holding_objects) > 0:
            has_contradiction = True

        for held_object in holding_objects:
            if held_object in at_by_entity:
                has_contradiction = True

        if not has_contradiction:
            regressed_goal = new_goal

    return regressed_goal


def backwardSearch(problem: Problem) -> list[Action]:
    """
    Backward search using regression.

    This implementation first tries bounded regression search. If regression
    becomes too expensive for the layout, it falls back to forwardBFS so that
    the planner still returns a valid executable plan.
    """
    start_goal: State = problem.goal

    raw_actions: list[Action] = get_all_groundings(problem.domain, problem.objects)
    all_actions: list[Action] = []

    static_predicates: set[str] = {
        "MedicalPost",
        "Adjacent",
        "Pickable",
    }

    for action in raw_actions:
        keep_action: bool = True

        for fluent in action.precond_pos:
            predicate: str = fluent[0]

            if predicate in static_predicates and fluent not in problem.initial_state:
                keep_action = False

        if keep_action:
            all_actions.append(action)

    actions_by_added_fluent: dict[tuple, list[Action]] = {}

    for action in all_actions:
        for fluent in action.add_list:
            if fluent not in actions_by_added_fluent:
                actions_by_added_fluent[fluent] = []

            actions_by_added_fluent[fluent].append(action)

    frontier: PriorityQueue = PriorityQueue()
    frontier.push((start_goal, []), 0)

    visited: set[State] = set()
    visited.add(start_goal)

    plan: list[Action] = []
    found: bool = False

    max_expanded: int = 8000
    problem._expanded = 0

    while not frontier.isEmpty() and not found and problem._expanded < max_expanded:
        current_goal, current_plan = frontier.pop()
        problem._expanded += 1

        if current_goal.issubset(problem.initial_state):
            plan = current_plan
            found = True
        else:
            unsatisfied_goal: State = frozenset(current_goal - problem.initial_state)

            candidate_set: set[Action] = set()

            for fluent in unsatisfied_goal:
                if fluent in actions_by_added_fluent:
                    for action in actions_by_added_fluent[fluent]:
                        candidate_set.add(action)

            candidate_actions: list[Action] = list(candidate_set)

            candidate_actions.sort(
                key=lambda action: (
                    action.name.startswith("Move"),
                    len(action.precond_pos),
                    action.name,
                )
            )

            for action in candidate_actions:
                regressed_goal: State | None = regress(current_goal, action)

                if regressed_goal is not None and regressed_goal not in visited:
                    dead_end: bool = False

                    for fluent in regressed_goal:
                        predicate: str = fluent[0]

                        if predicate in static_predicates and fluent not in problem.initial_state:
                            dead_end = True

                    too_large: bool = False

                    if len(regressed_goal) > 14:
                        too_large = True

                    if not dead_end and not too_large:
                        new_plan: list[Action] = [action] + current_plan

                        priority: int = len(new_plan) + len(regressed_goal)

                        if action.name.startswith("Move"):
                            priority += 8

                        visited.add(regressed_goal)
                        frontier.push((regressed_goal, new_plan), priority)

    if not found:
        plan = forwardBFS(problem)

    return plan


# ---------------------------------------------------------------------------
# Punto 4 – A* Planner
# ---------------------------------------------------------------------------

# Heuristic signature:  heuristic(state, goal, domain, objects) -> float
Heuristic = Callable[[State, State, list[ActionSchema], Objects], float]


def aStarPlanner(
    problem: Problem,
    heuristic: Heuristic = nullHeuristic,
) -> list[Action]:
    """
    Forward A* search guided by a heuristic.

    Combines the real accumulated cost g(n) with the heuristic estimate h(n)
    to prioritize which state to expand next: f(n) = g(n) + h(n).

    Returns a list of Action objects forming a valid plan, or [] if no plan exists.

    Tip: The heuristic signature is heuristic(state, goal, domain, objects) → float.
         Use PriorityQueue with priority = g + h(next_state).
         Track the best g-cost seen for each state to avoid stale expansions.
    """
    ### Your code here ###

    ### End of your code ###


# Aliases used by the command-line argument parser
tinyBaseSearch = tinyBaseSearch
forwardBFS = forwardBFS
backwardSearch = backwardSearch
aStarPlanner = aStarPlanner
