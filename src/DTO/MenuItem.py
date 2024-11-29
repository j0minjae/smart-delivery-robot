from dataclasses import dataclass
from typing import Optional

@dataclass
class MenuItem:
    menu_id: int
    quantity: int
    is_checked: Optional[bool] = None