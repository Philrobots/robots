import { ColorFactory } from '@/factories/ColorFactory';
import { CornerFactory } from '@/factories/CornerFactory';
import { PuckStateFactory } from '@/factories/PuckStateFactory';
import { Puck } from '@/types/puck';

// TODO : Weirdly could not use node-factory directly (same for PuckListFactory)
export class PuckFactory {
  static make = () =>
    new Puck(ColorFactory.get(), CornerFactory.get(), PuckStateFactory.get());
}
