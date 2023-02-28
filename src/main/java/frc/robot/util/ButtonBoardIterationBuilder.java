package frc.robot.util;

import java.util.function.Consumer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Buttonboard.Alphabet;

public class ButtonBoardIterationBuilder {
    Buttonboard buttonBoard;
    Buttonboard.Alphabet from = Alphabet.A;
    Buttonboard.Alphabet to = Alphabet.Z;
    // ArrayList<Buttonboard.Alphabet> excluding = new ArrayList<>();
    

    int fromCol = 0;
    int toCol = buttonBoard.width;


    public ButtonBoardIterationBuilder(Buttonboard bb) {
        this.buttonBoard = bb;
        to = Buttonboard.getAlphabetFromIndex(bb.width);
    }

    public ButtonBoardIterationBuilder FromRow(Buttonboard.Alphabet from) {
        this.from = from;
        return this;
    }
    
    public ButtonBoardIterationBuilder ToRow(Buttonboard.Alphabet to) {
        this.to = to;
        return this;
    }

    public ButtonBoardIterationBuilder FromColumn(int from) {
        this.fromCol = from;
        return this;
    }
    
    public ButtonBoardIterationBuilder ToColumn(int to) {
        this.toCol = to;
        return this;
    }

    public ButtonBoardIterationBuilder ThenForEach(Consumer<IterationInfo> foreach) {
        for(int i = from.getIndex(); i < to.getIndex(); i++) {
            for(int j = fromCol; j < toCol; j++) {
                Alphabet letter = Buttonboard.getAlphabetFromIndex(i);
                foreach.accept(new IterationInfo(letter, j, buttonBoard.buttonAt(letter, j)));
            }
        }

        return this;
    }

    public Buttonboard AndThen() {
        return buttonBoard;
    }

    public record IterationInfo (
        Alphabet row,
        int column,
        Trigger trigger
    ) {}
}
