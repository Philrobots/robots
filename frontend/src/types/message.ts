import { Color } from '@/types/color';
import { Corner } from '@/types/corner';
import { Coordinate } from '@/types/coordinate';
import { RobotConsumption } from './robotConsumption';

export interface Message {
  resistance?: number;
  tableImage?: string;
  robotConsumption?: RobotConsumption;
  puckColors?: Array<Color>;
  puckFirstCorner?: Corner;
  plannedTrajectoryCoordinates?: Array<Coordinate>;
  realTrajectoryCoordinate?: Coordinate;
  puckInGrip?: boolean;
  currentStep?: string;
}
