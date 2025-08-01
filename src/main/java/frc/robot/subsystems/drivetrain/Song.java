package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.IntStream;

public class Song {
    private static final Random random = new Random();
    private static final Timer songTimer = new Timer();
    private final int[] trackWeights;
    private final String filename;

    public static final Song[] startupSongs = new Song[] {
            new Song("mario.chrp", new int[] {5, 2, 1}),
            new Song("underground-2.chrp"),
            new Song("your-phone-linging.chrp", new int[] {2, 2, 2, 2}),
            new Song("candyland.chrp"),
            new Song("mii-theme.chrp"),
            new Song("richh-ballin.chrp"),
            new Song("tombstone.chrp"),
            new Song("nggyu.chrp", new int[] {6, 2}),
    };

    public static final Song[] disableSongs = new Song[] {
            new Song("mario-death.chrp", new int[] {3, 3, 2}),
            new Song("castle-complete.chrp", new int[] {2, 2, 1, 1, 1, 1}),
            new Song("level-complete.chrp", new int[] {3, 3, 2}),
            new Song("mario.chrp", new int[] {5, 2, 1}),
            new Song("underground-2.chrp"),
            new Song("candyland.chrp"),
            new Song("mii-theme.chrp"),
            new Song("richh-ballin.chrp"),
            new Song("tombstone.chrp"),
            new Song("nggyu.chrp", new int[] {6, 2}),
    };

    public Song(String filename) {
        this(filename, new int[] {8});
    }

    public Song(String filename, int[] trackWeights) {
        this.trackWeights = trackWeights;
        this.filename = filename;
    }

    private String getPath() {
        return "sound/" + filename;
    }

    public static boolean tenSeconds() {
        return songTimer.hasElapsed(10);
    }

    public static void playRandom(Swerve swerve, Song[] songs) {
        songs[random.nextInt(songs.length)].play(swerve);
    }

    private void play(Swerve swerve) {
        songTimer.restart();
        swerve.orchestra.stop();

        List<ParentDevice> motors = new ArrayList<>();

        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : swerve.getModules()) {
            module.getDriveMotor().getConfigurator().apply(Constants.SwerveConstants.AUDIO_CONFIGS, 0.05);
            module.getSteerMotor().getConfigurator().apply(Constants.SwerveConstants.AUDIO_CONFIGS, 0.05);

            motors.add(module.getDriveMotor());
            motors.add(module.getSteerMotor());
        }

        swerve.orchestra.clearInstruments();
        swerve.orchestra.loadMusic(getPath());

        IntStream.range(0, trackWeights.length).forEach(
                weightIndex -> IntStream.range(0, trackWeights[weightIndex]).forEach(
                        i -> swerve.orchestra.addInstrument(motors.remove(0), weightIndex)
                )
        );

        swerve.orchestra.play();
    }
}
