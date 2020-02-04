package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Button;

public class CustomButton extends Button {
	public CustomButton(BooleanSupplier supplier) {
		super(supplier);
	}
}