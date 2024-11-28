from dataclasses import dataclass
from .MenuItem import MenuItem

@dataclass
class OrderTicket:
    table_id: int
    elapsed: int
    order: list[MenuItem]