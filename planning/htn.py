from __future__ import annotations

from planning.pddl import Action, Problem, apply_action, is_applicable
from planning.utils import Queue

# ---------------------------------------------------------------------------
# HTN Infrastructure
# ---------------------------------------------------------------------------


class HLA:
    """
    A High-Level Action (HLA) in HTN planning.

    An HLA is an abstract task that can be refined into sequences of
    more primitive actions (or other HLAs). Each refinement is a list
    of HLA or Action objects.

    name:        Human-readable name for display
    refinements: List of possible refinements, each a list of HLA/Action objects
    """

    def __init__(self, name: str, refinements: list[list] | None = None) -> None:
        self.name = name
        self.refinements = refinements or []

    def __repr__(self) -> str:
        return f"HLA({self.name})"


def is_primitive(action: Action | HLA) -> bool:
    """Return True if action is a primitive (grounded Action), False if it is an HLA."""
    return isinstance(action, Action)


def is_plan_primitive(plan: list[Action | HLA]) -> bool:
    """Return True if every step in the plan is a primitive action."""
    return all(is_primitive(step) for step in plan)


# ---------------------------------------------------------------------------
# Punto 5a – hierarchicalSearch
# ---------------------------------------------------------------------------


def hierarchicalSearch(problem: Problem, hlas: list[HLA]) -> list[Action]:
    """
    HTN planning via BFS over hierarchical plan refinements.

    Start with an initial plan containing a single top-level HLA.
    At each step, find the first non-primitive step in the plan and
    replace it with one of its refinements. Continue until the plan
    is fully primitive and achieves the goal when executed from the
    initial state.

    Returns a list of primitive Action objects, or [] if no plan found.

    Tip: The search space consists of (partial plan, current plan index) pairs.
         Use a Queue (BFS) to explore all refinement choices fairly.
         A plan is a solution when:
           1. It contains only primitive actions (is_plan_primitive), AND
           2. Executing it from the initial state reaches a goal state.
         To simulate execution, apply each action in order using apply_action().
    """
    ### Your code here ###
    initial_plan: list = list(hlas) if isinstance(hlas, list) else [hlas]

    frontier: Queue = Queue()
    frontier.push(initial_plan)

    visited: set[tuple[str, ...]] = set()

    result: list[Action] = []
    found: bool = False

    while not frontier.isEmpty() and not found:
        plan = frontier.pop()

        key: tuple[str, ...] = tuple(step.name for step in plan)

        if key not in visited:
            visited.add(key)

            if is_plan_primitive(plan):
                # Validación: ejecutar el plan desde el estado inicial.
                state = problem.getStartState()
                valid: bool = True
                i: int = 0

                while i < len(plan) and valid:
                    action = plan[i]

                    if is_applicable(state, action):
                        state = apply_action(state, action)
                        i += 1
                    else:
                        valid = False

                if valid and problem.isGoalState(state):
                    result = list(plan)
                    found = True
            else:
                # Hay al menos una HLA. Encontrar la primera y refinarla.
                idx: int = -1
                j: int = 0

                while j < len(plan) and idx == -1:
                    if not is_primitive(plan[j]):
                        idx = j
                    else:
                        j += 1

                hla = plan[idx]

                for refinement in hla.refinements:
                    new_plan = plan[:idx] + list(refinement) + plan[idx + 1:]
                    frontier.push(new_plan)

    return result
    ### End of your code ###


# ---------------------------------------------------------------------------
# Punto 5b – HLA Definitions
# ---------------------------------------------------------------------------


def build_htn_hierarchy(problem: Problem) -> list[HLA]:
    """
    Build HTN HLAs for the rescue domain.

    The hierarchy defines four HLA types:
      - Navigate(from, to):       Move the robot step by step from one cell to another
      - PrepareSupplies(s, m):    Collect supplies and set them up at the medical post
      - ExtractPatient(p, m):     Pick up the patient and bring them to the medical post
      - FullRescueMission(s,p,m): Complete one rescue: prepare supplies + extract + rescue

    Refinements are built from the ground state to generate concrete Action objects.

    Tip: Refinements for Navigate are all single-step Move sequences between
         adjacent cells. PrepareSupplies and ExtractPatient chain Navigate HLAs
         with primitive PickUp, SetupSupplies, PutDown, and Rescue actions.
    """
def build_htn_hierarchy(problem: Problem) -> list[HLA]:
    """
    Construye la jerarquía HTN para el dominio de rescate.

    HLAs definidas:
      - Navigate(from, to)         : mueve el robot entre dos celdas
      - PrepareSupplies(s, m, start): recoge suministros y los instala en m
      - ExtractPatient(p, m, start) : recoge al paciente y lo lleva a m
      - FullRescueMission(s,p,m,st) : misión completa de rescate

    Decisión de diseño para Navigate:
      En vez de la refinación recursiva clásica (Navigate(a,b) → [Move(a,c),
      Navigate(c,b)] para cada vecino c), que tiene un factor de ramificación
      exponencial, usamos UN solo refinamiento por par (a, b): la secuencia
      de Moves del camino más corto, calculado con BFS sobre la cuadrícula.

      Esto preserva la estructura jerárquica conceptual y garantiza que el
      tramo de movimiento es óptimo en longitud. Trade-off: no se exploran
      rutas alternativas. Para este dominio (cuadrícula con un solo robot
      y sin restricciones dinámicas en celdas), no hay pérdida de
      completitud porque cualquier ruta válida tendrá longitud ≥ la del
      camino más corto.

    Decisión de diseño para FullRescueMission:
      Dos refinamientos:
        A) Misión completa: PrepareSupplies + ExtractPatient + Rescue.
        B) Misión abreviada: ExtractPatient + Rescue (suministros ya listos).

      La precondición negativa de SetupSupplies sobre SuppliesReady fuerza
      que el refinamiento A solo sea ejecutable la PRIMERA vez en un puesto
      médico dado. Las misiones posteriores en el mismo puesto deben usar B.
      hierarchicalSearch encuentra automáticamente esta combinación.
    """
    init = problem.initial_state
    objects = problem.objects
    schemas: dict[str, object] = {s.name: s for s in problem.domain}

    # --- Constantes del problema ---------------------------------------------
    robot = objects["robots"][0]
    cells: list = list(objects["cells"])
    supplies: list = list(objects["supplies"])
    patients: list = list(objects["patients"])
    medical_posts: list = list(objects["medical_posts"])

    hierarchy: list[HLA] = []

    if patients and medical_posts and supplies:
        #  iniciales en lista de adyacencia (desde el estado inicial).
        locations: dict = {}
        adjacency: dict = {c: [] for c in cells}

        for f in init:
            if f[0] == "At":
                locations[f[1]] = f[2]
            elif f[0] == "Adjacent":
                adjacency.setdefault(f[1], []).append(f[2])

        robot_init = locations[robot]
        m = medical_posts[0]
        s_used = supplies[0]  # Soo se nedcesita un suministro para todo

       # Recorrer con el camino mas corto (dikstra0)
       #Codigo sacado de STACKOVERFLOW-- nO ES DE MI AUTORIA. ADAPTAFO PARA USAR QUEUE EN UTILS.py
        def shortest_path(start, end) -> list[tuple] | None:
            """Retorna una lista de aristas (from, to) o None si no hay ruta."""
            result_path: list[tuple] | None = None

            if start == end:
                result_path = []
            else:
                parent: dict = {start: None}
                q: Queue = Queue()
                q.push(start)
                found_target: bool = False

                while not q.isEmpty() and not found_target:
                    u = q.pop()

                    if u == end:
                        found_target = True
                    else:
                        for v in adjacency.get(u, []):
                            if v not in parent:
                                parent[v] = u
                                q.push(v)

                if found_target:
                    edges: list[tuple] = []
                    cur = end
                    while parent[cur] is not None:
                        edges.append((parent[cur], cur))
                        cur = parent[cur]
                    edges.reverse()
                    result_path = edges

            return result_path
        
        #YA NO MAS STACK

        
        def prim(schema_name: str, **binding) -> Action:
            return schemas[schema_name].ground(binding)

        nav_cache: dict[tuple, HLA] = {}

        def navigate(a, b) -> HLA:
            if (a, b) in nav_cache:
                h = nav_cache[(a, b)]
            else:
                h = HLA(f"Navigate({a}->{b})")
                path = shortest_path(a, b)

                if path is None:
                    h.refinements = []  # destino inalcanzable
                elif len(path) == 0:
                    h.refinements = [[]]  # ya estamos allá
                else:
                    moves = [
                        prim("Move", r=robot, from_cell=fr, to_cell=to)
                        for (fr, to) in path
                    ]
                    h.refinements = [moves]

                nav_cache[(a, b)] = h

            return h

        def prepare_supplies(s, post, start) -> HLA:
            sloc = locations[s]
            h = HLA(f"PrepareSupplies({s},{post},from={start})")
            h.refinements = [[
                navigate(start, sloc),
                prim("PickUp", r=robot, obj=s, loc=sloc),
                navigate(sloc, post),
                prim("SetupSupplies", r=robot, s=s, loc=post),
            ]]
            return h

        def extract_patient(p, post, start) -> HLA:
            ploc = locations[p]
            h = HLA(f"ExtractPatient({p},{post},from={start})")
            h.refinements = [[
                navigate(start, ploc),
                prim("PickUp", r=robot, obj=p, loc=ploc),
                navigate(ploc, post),
                prim("PutDown", r=robot, obj=p, loc=post),
            ]]
            return h

        def full_rescue_mission(s, p, post, start) -> HLA:
            h = HLA(f"FullRescueMission({s},{p},{post},from={start})")
            h.refinements = [
                [   # Refinamiento A: misión completa
                    prepare_supplies(s, post, start),
                    extract_patient(p, post, post),
                    prim("Rescue", r=robot, p=p, loc=post),
                ],
                [   # Refinamiento B: misión abreviada (suministros ya listos)
                    extract_patient(p, post, start),
                    prim("Rescue", r=robot, p=p, loc=post),
                ],
            ]
            return h

       
        hierarchy.append(full_rescue_mission(s_used, patients[0], m, robot_init))

        for p in patients[1:]:
            hierarchy.append(full_rescue_mission(s_used, p, m, m))

    return hierarchy
