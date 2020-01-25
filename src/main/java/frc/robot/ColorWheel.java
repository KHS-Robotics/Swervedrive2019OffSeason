package frc.robot;

public enum ColorWheel {
    YELLOW(1), RED(2), GREEN(3), BLUE(4);
    
    public final int signature;
    private ColorWheel(int signature) {
        this.signature = signature;
    }

    public boolean isYellow() {
        return this.signature == YELLOW.signature;
    }

    public boolean isRed() {
        return this.signature == RED.signature;
    }

    public boolean isGreen() {
        return this.signature == GREEN.signature;
    }

    public boolean isBlue() {
        return this.signature == BLUE.signature;
    }

    public static ColorWheel toColor(int signature) {
        switch(signature) {
            case 1:
                return YELLOW;
            case 2:
                return RED;
            case 3:
                return GREEN;
            case 4:
                return BLUE;
            default:
                throw new IllegalArgumentException();
        }
    }
}