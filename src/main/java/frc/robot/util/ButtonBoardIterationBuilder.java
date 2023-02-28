package frc.robot.util;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Buttonboard.Alphabet;

public class ButtonBoardIterationBuilder {
    Buttonboard buttonBoard;
    Alphabet from = Alphabet.A;
    Alphabet to = Alphabet.Z;
    // ArrayList<Buttonboard.Alphabet> excluding = new ArrayList<>();

    int fromCol = 1;
    int toCol;

    public ButtonBoardIterationBuilder(Buttonboard bb) {
        this.buttonBoard = bb;
        to = Alphabet.fromIndex(bb.height);
        toCol = bb.width;
    }

    public ButtonBoardIterationBuilder fromRow(Alphabet from) {
        this.from = from;
        return this;
    }
    
    public ButtonBoardIterationBuilder toRow(Alphabet to) {
        this.to = to;
        return this;
    }

    public ButtonBoardIterationBuilder fromColumn(int from) {
        this.fromCol = from;
        return this;
    }
    
    public ButtonBoardIterationBuilder toColumn(int to) {
        this.toCol = to;
        return this;
    }

    public ButtonBoardIterationBuilder forEach(Consumer<IterationInfo> foreach) {
        for(int i = from.getIndex(); i <= to.getIndex(); i++) {
            for(int j = fromCol; j <= toCol; j++) {
                Alphabet letter = Alphabet.fromIndex(i);
                foreach.accept(new IterationInfo(letter, j, buttonBoard.buttonAt(letter, j)));
            }
        }

        return this;
    }

    public Buttonboard getButtonboard() {
        return buttonBoard;
    }

    public record IterationInfo (
        Alphabet row,
        int column,
        Trigger trigger
    ) {}
}
