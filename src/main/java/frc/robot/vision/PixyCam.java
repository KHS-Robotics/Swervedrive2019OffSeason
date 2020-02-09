package frc.robot.vision;

import java.util.ArrayList;

import frc.robot.RobotContainer;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class PixyCam {
	private static ArrayList<Block> blocks = new ArrayList<>();
	private static ArrayList<ColorBlock> returnBlocks = new ArrayList<>();
	private static ArrayList<Integer> sigs = new ArrayList<>();
	private static ArrayList<Integer> currentIdxs = new ArrayList<>();

	public static ArrayList<ColorBlock> sortByCenter(ArrayList<ColorBlock> args) {
		args.sort(
				(a, b) -> Double.compare(Math.abs(ColorBlock.xHalf - a.getX()) + Math.abs(ColorBlock.yHalf - a.getY()),
						Math.abs(ColorBlock.yHalf - b.getX()) + Math.abs(ColorBlock.yHalf - b.getY())));
		return args;
	}

	public static ArrayList<ColorBlock> sortByWeight(ArrayList<ColorBlock> args) {
		args.sort((a, b) -> Double.compare(a.getWeight(), b.getWeight()));
		return args;
	}

	public static ColorBlock averageDupBlocks(ArrayList<Integer> sigs, ArrayList<Block> blocks) {
		double totalWidth = 0, totalHeight = 0, xAverage = 0, yAverage = 0, smallX, largeX, smallY, largeY;

		for (int i = 0; i < sigs.size(); i++) {
			totalWidth += blocks.get(sigs.get(i)).getWidth();
			totalHeight += blocks.get(sigs.get(i)).getHeight();
		}

		int curBlockX = blocks.get(0).getX();
		int curBlockY = blocks.get(0).getY();

		smallX = curBlockX - (blocks.get(0).getWidth() / 2.0);
		largeX = curBlockX + (blocks.get(0).getWidth() / 2.0);

		smallY = curBlockY - (blocks.get(0).getHeight() / 2.0);
		largeY = curBlockY + (blocks.get(0).getHeight() / 2.0);

		for (int i = 0; i < sigs.size(); i++) {
			xAverage += (blocks.get(sigs.get(i)).getWidth() / totalWidth) * blocks.get(i).getX();
			yAverage += (blocks.get(sigs.get(i)).getHeight() / totalHeight) * blocks.get(i).getY();

			double newXMinus = blocks.get(i).getX() - (blocks.get(i).getWidth() / 2.0);
			double newXPlus = blocks.get(i).getX() + (blocks.get(i).getWidth() / 2.0);

			double newYMinus = blocks.get(i).getY() - (blocks.get(i).getHeight() / 2.0);
			double newYPlus = blocks.get(i).getY() + (blocks.get(i).getHeight() / 2.0);

			if (newXMinus < smallX) {
				smallX = newXMinus;
			}
			if (newXPlus > largeX) {
				largeX = newXPlus;
			}

			if (newYMinus < smallY) {
				smallY = newYMinus;
			}
			if (newYPlus > largeY) {
				largeY = newYPlus;
			}
		}

		double width = largeX - smallX, height = largeY - smallY;

		return new ColorBlock(xAverage, yAverage, width, height, blocks.get(sigs.get(0)).getSignature());
	}

	public static ArrayList<ColorBlock> getBlocks() {
		Pixy2 pixy = RobotContainer.pixy;
		pixy.getCCC().getBlocks(true, 255, 8);

		blocks = pixy.getCCC().getBlocks();

		returnBlocks.clear();

		sigs.clear();

		for (int i = 0; i < blocks.size(); i++) {
			sigs.add(blocks.get(i).getSignature());
		}

		for (int sig = 1; sig <= 4; sig++) {
			currentIdxs.clear();
			for (int i = 0; i < sigs.size(); i++) {
				if (sigs.get(i) == sig) {
					currentIdxs.add(i);
				}
				if (currentIdxs.size() > 0) {
					returnBlocks.add(averageDupBlocks(currentIdxs, blocks));
				}
			}
		}

		return returnBlocks;
	}
}