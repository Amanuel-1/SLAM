from abc import ABC,abstractmethod
from typing import List,Tuple
from utils.geometry import *
class LineExtraction(ABC):

    """
        - this is a base class for all line extraction algorithm
    """
    @abstractmethod
    def extract(point_cloud:List[Tuple]):
        "pass"

class SplitMergeExtraction(LineExtraction):


    def extract(self,point_cloud:List[Tuple]):
        "pass"
    
    def detect_seed_segments(self,position,break_point_index):
        flag = True
        self.NP = max(0,self.NP)
        self.seed_segments = []

        for i in range(break_point_index,(self.NP-self.PMIN)):
            pred_ponits = []
            j= i+ self.SNUM

            m,b = odr_fit(self.laser_points[i:j])
            line_params = slope_intersept_to_general(m,b)

            for k in range(i,j):
                point,_ = self.laser_points[k]
                predicted_point = get_predicted_point(line_params,point,position)

                d = point_point_distance(predicted_point,point)
                pred_ponits.append(predicted_point)
                if d > self.DELTA:
                    flag = False
                    break
                
                #check the distance between the actual point and the fitted line
                d2 = point_line_distance(point,line_params)

                if d2 > self.EPSILON:
                    flag = False
                    break

                if flag:
                    self.Params = line_params
                    return [self.laser_points[i:j],pred_ponits,(i,j)]
                
                
        return None

    

    def seed_segment_growing(self,indices,break_point):
        

        line = self.line_params
        i,j = indices

        pi,pf = max(break_point,i-1),min(j+1,len(self.laser_points)-1)
        point = self.laser_points[pf][0]

        while point_line_distance(line,self.laser_points[pf][0] < self.EPSILON):
            if pf> self.NP -1:
                break
            else:
                m,b = odr_fit(self.laser_points[pi:pf])
                line = slope_intersept_to_general(m,b)
                line_params = slope_intersept_to_general(m, b)
                point = self.laser_points[pf][0]


            pf +=1
            next_point = self.laser_points[pf][0]
            if point_point_distance(point,next_point)>self.GMAX:
                break
        
        pf -=1
        while point_line_distance(line,self.laser_points[pi][0]):
            if pi < break_point:
                break
            else:
                m,b = odr_fit(self.laser_points[pi:pf])
                line = slope_intersept_to_general(m,b)
                point = self.laser_points[pi][0]
            pi -=1
            next_point = self.laser_points[pi][0]
            if point_point_distance(point,next_point)>self.GMAX:
                break
        pi+=1

        LR = point_point_distance(self.laser_points[pi][0],self.laser_points[pf][0])
        PR = len(self.laser_points[pi:pf])

        if (LR >= self.LMIN) and (PR >= self.PMIN):
            self.line_params =line
            m,b = general_to_slope_intercept(*line)
            self.two_points = get_line_points(m,b)
            self.line_segments.append(self.laserpoints[pi +1][0] , self.laser_points[pf-1][0])

            return [self.laser_points[pi:pf],self.two_points,(self.laser_points[pi+1][0],self.laser_points[pf-1][0]),pf,line,(m,b)]

        else:
            return None






class IncrementalExtraction(LineExtraction):

    def _check_line_condition(line_model):
        "pass"

    def extract(point_cloud:List[Tuple]):
        "pass"