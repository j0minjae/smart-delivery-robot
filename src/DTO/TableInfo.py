from dataclasses import dataclass, field
from datetime import datetime
from .MenuItem import MenuItem

@dataclass
class TableInfo:
    table_id: int
    status: bool = False
    arrival_time: datetime = None 
    order: list[MenuItem] = field(default_factory=list)
    payment: int = 0