import { PuckList } from '@/types/puckList';
import { PuckFactory } from '@/factories/PuckFactory';
import { PuckState } from '@/types/puckState';

export class PuckListFactory {
  static make = () => {
    const pucks = [];
    for (let index = 0; index < PuckList.PUCKS_COUNT; index++) {
      pucks.push(PuckFactory.make());
    }
    return new PuckList(pucks);
  };

  static makeWithStates = (puckStates: Array<PuckState>) => {
    if (puckStates.length != PuckList.PUCKS_COUNT) return new PuckList();
    const puckList = PuckListFactory.make();

    puckStates.forEach((state, index) => (puckList.get(index).state = state));

    return puckList;
  };
}
