import { factory } from 'node-factory';
import { Coordinate } from '@/types/coordinate';

export const CoordinateFactory = factory<Coordinate>((fake) => ({
  // TODO : Those numbers should be stocked elsewhere
  x: fake.random.number(1600),
  y: fake.random.number(904),
}));
