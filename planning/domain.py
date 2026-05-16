from __future__ import annotations

from planning.pddl import ActionSchema

# ---------------------------------------------------------------------------
# Punto 1a – Complete the preconditions and effects of each action schema.
#
# Each schema uses string variable names as placeholders:
#   "r"          → the robot
#   "from_cell"  → source cell
#   "to_cell"    → destination cell
#   "obj"        → any pickable object
#   "s"          → medical supplies
#   "p"          → patient
#   "loc"        → a cell
#
# Fluent templates are tuples whose elements are either variable names or
# literal constant strings. get_applicable_actions() will substitute
# variable names with real constants during grounding.
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Move(r, from_cell, to_cell)
# Move the robot one step to an adjacent, free cell.
# ---------------------------------------------------------------------------

MOVE: ActionSchema = ActionSchema(
    name="Move",
    parameters=["r", "from_cell", "to_cell"],
    precond_pos=[
        ("At", "r", "from_cell"),
        ("Adjacent", "from_cell", "to_cell"),
        ("Free", "to_cell"),
    ],
    precond_neg=[],
    add_list=[
        ("At", "r", "to_cell"),
        ("Free", "from_cell"),
    ],
    del_list=[
        ("At", "r", "from_cell"),
        ("Free", "to_cell"),
    ],
)


# ---------------------------------------------------------------------------
# PickUp(r, obj, loc)
# Pick up a pickable object at the robot's current cell.
# ---------------------------------------------------------------------------

PICKUP: ActionSchema = ActionSchema(
    name="PickUp",
    parameters=["r", "obj", "loc"],
    precond_pos=[
        ("At", "r", "loc"),
        ("At", "obj", "loc"),
        ("HandsFree", "r"),
        ("Pickable", "obj"),
    ],
    precond_neg=[],
    add_list=[
        ("Holding", "r", "obj"),
    ],
    del_list=[
        ("At", "obj", "loc"),
        ("HandsFree", "r"),
    ],
)


# ---------------------------------------------------------------------------
# PutDown(r, obj, loc)
# Place a held object at the robot's current cell.
# ---------------------------------------------------------------------------

PUTDOWN: ActionSchema = ActionSchema(
    name="PutDown",
    parameters=["r", "obj", "loc"],
    precond_pos=[
        ("At", "r", "loc"),
        ("Holding", "r", "obj"),
    ],
    precond_neg=[],
    add_list=[
        ("At", "obj", "loc"),
        ("HandsFree", "r"),
    ],
    del_list=[
        ("Holding", "r", "obj"),
    ],
)


# ---------------------------------------------------------------------------
# Rescue(r, p, loc)
# Rescue a patient who is at a medical post where supplies are ready.
# ---------------------------------------------------------------------------

RESCUE: ActionSchema = ActionSchema(
    name="Rescue",
    parameters=["r", "p", "loc"],
    precond_pos=[
        ("At", "r", "loc"),
        ("At", "p", "loc"),
        ("MedicalPost", "loc"),
        ("SuppliesReady", "loc"),
    ],
    precond_neg=[],
    add_list=[
        ("Rescued", "p"),
    ],
    del_list=[
        ("At", "p", "loc"),
    ],
)


# ---------------------------------------------------------------------------
# SetupSupplies(r, s, loc)
# Set up medical supplies at a medical post.
# ---------------------------------------------------------------------------

SETUP_SUPPLIES: ActionSchema = ActionSchema(
    name="SetupSupplies",
    parameters=["r", "s", "loc"],
    precond_pos=[
        ("At", "r", "loc"),
        ("MedicalPost", "loc"),
        ("Holding", "r", "s"),
    ],
    precond_neg=[
        ("SuppliesReady", "loc"),
    ],
    add_list=[
        ("SuppliesReady", "loc"),
        ("HandsFree", "r"),
    ],
    del_list=[
        ("Holding", "r", "s"),
    ],
)


DOMAIN: list[ActionSchema] = [
    MOVE,
    PICKUP,
    PUTDOWN,
    RESCUE,
    SETUP_SUPPLIES,
]

