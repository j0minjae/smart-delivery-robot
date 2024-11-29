from dataclasses import dataclass, field
from .MenuItem import MenuItem

@dataclass
class OrderTicket:
    table_id: int
    elapsed: int=0
    order: list[MenuItem] = field(default_factory=list)