import { Color } from '@/types/color';
import { PuckState } from '@/types/puckState';
import { Corner } from '@/types/corner';

export class Puck {
  color: Color;
  corner: Corner;
  state: PuckState;
  number = 0;

  constructor(
    color: Color = Color.Unset,
    corner: Corner = Corner.UNSET,
    state: PuckState = PuckState.UNTOUCHED
  ) {
    this.color = color;
    this.corner = corner;
    this.state = state;
  }

  get isUntouched(): boolean {
    return this.state === PuckState.UNTOUCHED;
  }

  get isGripped(): boolean {
    return this.state === PuckState.GRIPPED;
  }

  get isReleased(): boolean {
    return this.state === PuckState.RELEASED;
  }
}
