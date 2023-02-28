package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Buttonboard extends CommandGenericHID {
    int width;
    int height;
    static HashMap<Pair<Alphabet, Integer>, Trigger> cachedButtons = new HashMap<>();

    public Buttonboard(int port, int width, int height) {
        super(port);
        this.width = width;
    }

    public Trigger buttonAt(Alphabet vertical, int horizontal) {
        var pair = new Pair<>(vertical, horizontal);
        if (cachedButtons.containsKey(pair)) {
            return cachedButtons.get(pair);
        }

        // This starts it off as 0
        int y = vertical.index;
        int x = horizontal;
        if (x <= 0) {
            throw new IndexOutOfBoundsException("Number starts at 1, not 0");
        }
        if(x > width) {
            throw new IndexOutOfBoundsException("Number, " + x + " is larger than your button board width, " + width);
        }
        if(y > height) {
            throw new IndexOutOfBoundsException("Letter at index " + y + " is larger than your button board width, " + height);
        }

        Trigger foundTrigger = button(y * width + x);
        cachedButtons.put(pair, foundTrigger);

        return foundTrigger;
    }

    public ButtonBoardIterationBuilder Iterate() {
        return new ButtonBoardIterationBuilder(this);
    }

    public Trigger A1() {
        return buttonAt(Alphabet.A, 1);
    }

    public Trigger A2() {
        return buttonAt(Alphabet.A, 2);
    }

    public Trigger A3() {
        return buttonAt(Alphabet.A, 3);
    }

    public Trigger A4() {
        return buttonAt(Alphabet.A, 4);
    }

    public Trigger A5() {
        return buttonAt(Alphabet.A, 5);
    }

    public Trigger A6() {
        return buttonAt(Alphabet.A, 6);
    }

    public Trigger A7() {
        return buttonAt(Alphabet.A, 7);
    }

    public Trigger B1() {
        return buttonAt(Alphabet.B, 1);
    }

    public Trigger B2() {
        return buttonAt(Alphabet.B, 2);
    }

    public Trigger B3() {
        return buttonAt(Alphabet.B, 3);
    }

    public Trigger B4() {
        return buttonAt(Alphabet.B, 4);
    }

    public Trigger B5() {
        return buttonAt(Alphabet.B, 5);
    }

    public Trigger B6() {
        return buttonAt(Alphabet.B, 6);
    }

    public Trigger B7() {
        return buttonAt(Alphabet.B, 7);
    }

    public Trigger C1() {
        return buttonAt(Alphabet.C, 1);
    }

    public Trigger C2() {
        return buttonAt(Alphabet.C, 2);
    }

    public Trigger C3() {
        return buttonAt(Alphabet.C, 3);
    }

    public Trigger C4() {
        return buttonAt(Alphabet.C, 4);
    }

    public Trigger C5() {
        return buttonAt(Alphabet.C, 5);
    }

    public Trigger C6() {
        return buttonAt(Alphabet.C, 6);
    }

    public Trigger C7() {
        return buttonAt(Alphabet.C, 7);
    }

    public enum Alphabet {
        A(1, "A"), B(2, "B"), C(3, "C"), D(4, "D"), E(5, "E"), F(6, "F"), G(7, "G"), H(8, "H"), I(9, "I"), J(10, "J"), K(11, "K"), L(12, "L"), M(13, "M"), N(14, "N"), O(15, "O"), P(16, "P"), Q(17, "Q"), R(18, "R"), S(19, "S"), T(20, "T"), U(21, "U"), V(22, "V"), W(23, "W"), X(24, "X"), Y(25, "Y"), Z(26, "Z");
        
        private final int index;
        private final String letter;
        
        private Alphabet(int index, String letter) {
            this.index = index;
            this.letter = letter;
        }
        
        public int getIndex() {
            return index;
        }
        
        public String getLetter() {
            return letter;
        }
    }
    

    public static Alphabet getAlphabetFromIndex(int index) {
        for (Alphabet alphabet : Alphabet.values()) {
            if (alphabet.getIndex() == index) {
                return alphabet;
            }
        }
        throw new IllegalArgumentException("Invalid index: " + index);
    }
}
