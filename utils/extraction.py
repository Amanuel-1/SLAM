from abc import ABC,abstractmethod
from typing import List,Tuple
class LineExtraction(ABC):

    """
        - this is a base class for all line extraction algorithm
    """
    @abstractmethod
    def extract(point_cloud:List[Tuple]):
        "pass"


class IncrementalExtraction(LineExtraction):

    def _check_line_condition(line_model):
        

    def extract(point_cloud:List[Tuple]):
        "pass"