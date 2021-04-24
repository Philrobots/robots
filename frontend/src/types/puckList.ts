import { Puck } from '@/types/puck';
import { Color } from '@/types/color';
import { Corner, getNextCorner } from '@/types/corner';
import { PuckState } from '@/types/puckState';

export class PuckList {
  static readonly PUCKS_COUNT = 3;
  pucks: Array<Puck>;

  constructor(
    pucks: Array<Puck> = Array.from(
      { length: PuckList.PUCKS_COUNT },
      () => new Puck()
    )
  ) {
    pucks.forEach((puck, index) => (puck.number = index + 1));

    this.pucks = pucks;
  }

  get first(): Puck {
    return this.pucks[0];
  }

  get(index: number): Puck {
    return this.pucks[index];
  }

  get releasedPucks(): Array<Puck> {
    return this.pucks.filter((puck) => puck.isReleased);
  }

  get hasOneGripped(): boolean {
    return this.pucks.some((puck) => puck.isGripped);
  }

  set hasOneGripped(hasOneGripped: boolean) {
    if (hasOneGripped) {
      this.setPuckState(PuckState.UNTOUCHED, PuckState.GRIPPED);
    } else {
      this.setPuckState(PuckState.GRIPPED, PuckState.RELEASED);
    }
  }

  // TODO : Using Puck getters would be nice
  private setPuckState(oldState: PuckState, newState: PuckState) {
    const puck = this.pucks.find((puck) => puck.state === oldState);
    if (puck) {
      puck.state = newState;
    }
  }

  get colors(): Array<Color> {
    return this.pucks.map((puck) => puck.color);
  }

  set colors(colors: Array<Color>) {
    if (colors.length == PuckList.PUCKS_COUNT) {
      this.pucks.forEach((puck, index) => (puck.color = colors[index]));
    }
  }

  set firstCorner(corner: Corner) {
    this.first.corner = corner;
    let nextCorner = corner;

    for (let i = 1; i < PuckList.PUCKS_COUNT; i++) {
      nextCorner = getNextCorner(nextCorner);
      this.get(i).corner = nextCorner;
    }
  }
}
