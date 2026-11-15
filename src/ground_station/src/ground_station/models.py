"""Transport-independent mission models and grid planning logic."""

from typing import List, Sequence, Tuple


class GridMission:
    """Grid/keep-out model and deterministic column-wise coverage route."""

    def __init__(self, rows: int = 8, columns: int = 7):
        if rows < 1 or columns < 1:
            raise ValueError("Grid dimensions must be positive")
        self.rows = rows
        self.columns = columns
        self.no_fly_cells = set()  # zero-based (row, column)
        self.route: List[Tuple[int, int]] = []

    def parse_cell(self, code: str) -> Tuple[int, int]:
        value = str(code).strip().upper()
        if len(value) < 2 or not value[0].isalpha() or not value[1:].isdigit():
            raise ValueError("Cell code must be a letter followed by a number, for example A1")
        row = ord(value[0]) - ord("A")
        column = int(value[1:]) - 1
        if not (0 <= row < self.rows and 0 <= column < self.columns):
            raise ValueError("Cell code is outside the configured grid")
        return row, column

    def format_cell(self, cell: Tuple[int, int]) -> str:
        row, column = cell
        if not (0 <= row < self.rows and 0 <= column < self.columns):
            raise ValueError("Cell coordinate is outside the configured grid")
        return "{}{}".format(chr(ord("A") + row), column + 1)

    def cell_to_local(self, cell: Tuple[int, int], cell_size: float = 1.0) -> Tuple[float, float]:
        """Convert a cell center to local coordinates with H7 at the lower-right."""
        row, column = cell
        if not (0 <= row < self.rows and 0 <= column < self.columns):
            raise ValueError("Cell coordinate is outside the configured grid")
        return (float(self.columns - 1 - column) * cell_size, float(self.rows - 1 - row) * cell_size)

    def set_no_fly_cells(self, codes: Sequence[str], require_three: bool = False) -> None:
        normalized = [str(code).strip() for code in codes if str(code).strip()]
        if require_three and len(normalized) != 3:
            raise ValueError("Exactly three keep-out cells are required when strict mode is enabled")
        cells = [self.parse_cell(code) for code in normalized]
        if len(set(cells)) != len(cells):
            raise ValueError("Keep-out cells must be unique")
        if require_three and len(cells) == 3:
            rows = {cell[0] for cell in cells}
            columns = {cell[1] for cell in cells}
            if len(rows) == 1:
                indices = sorted(cell[1] for cell in cells)
            elif len(columns) == 1:
                indices = sorted(cell[0] for cell in cells)
            else:
                raise ValueError("Keep-out cells must be in one continuous row or column")
            if indices != list(range(indices[0], indices[0] + 3)):
                raise ValueError("Keep-out cells must be adjacent and continuous")
        self.no_fly_cells = set(cells)

    def update_no_fly_cells(self, add_codes: Sequence[str] = (), remove_codes: Sequence[str] = ()) -> None:
        add_cells = {self.parse_cell(str(code).strip()) for code in add_codes if str(code).strip()}
        remove_cells = {self.parse_cell(str(code).strip()) for code in remove_codes if str(code).strip()}
        self.no_fly_cells = (self.no_fly_cells | add_cells) - remove_cells

    def generate_route(self) -> List[Tuple[int, int]]:
        available = {
            (row, column)
            for row in range(self.rows)
            for column in range(self.columns)
            if (row, column) not in self.no_fly_cells
        }
        route = []
        targets = []
        for column in range(self.columns - 1, -1, -1):
            rows = range(self.rows - 1, -1, -1) if (self.columns - 1 - column) % 2 == 0 else range(self.rows)
            targets.extend((row, column) for row in rows if (row, column) in available)

        for target in targets:
            if not route:
                route.append(target)
                continue
            if target == route[-1]:
                continue
            previous = {route[-1]: None}
            queue = [route[-1]]
            for current in queue:
                row, column = current
                neighbors = ((row - 1, column), (row + 1, column), (row, column - 1), (row, column + 1))
                for neighbor in neighbors:
                    if neighbor in available and neighbor not in previous:
                        previous[neighbor] = current
                        queue.append(neighbor)
                        if neighbor == target:
                            queue = []
                            break
                if not queue:
                    break
            if target not in previous:
                raise ValueError("禁飞区将可巡查区域隔断，无法生成连续航线")
            segment = []
            current = target
            while current != route[-1]:
                segment.append(current)
                current = previous[current]
            route.extend(reversed(segment))
        self.route = route
        return list(route)
