package quadcopter.udooneo.neoquadcopter.View;

import android.os.Bundle;
import android.support.design.widget.FloatingActionButton;
import android.support.design.widget.Snackbar;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.view.View;
import android.widget.Button;
import android.widget.SeekBar;

import quadcopter.udooneo.neoquadcopter.QuadcopterApplication;
import quadcopter.udooneo.neoquadcopter.R;
import quadcopter.udooneo.neoquadcopter.model.Model;

public class controlActivity extends AppCompatActivity implements View.OnClickListener{

    Model model = null;
    int i = 0;

    SeekBar s1;
    SeekBar s2;
    SeekBar s3;
    SeekBar s4;

    Button sendmessage;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_control);

        model = ((QuadcopterApplication) this.getApplication()).getModel();

        sendmessage = (Button) findViewById(R.id.controlview_button);
        sendmessage.setOnClickListener(this);

        s1 = (SeekBar)findViewById(R.id.seekBar1);
        s2 = (SeekBar)findViewById(R.id.seekBar2);
        s3 = (SeekBar)findViewById(R.id.seekBar3);
        s4 = (SeekBar)findViewById(R.id.seekBar4);
    }

    @Override
    public void onClick(View v) {
        //System.out.println("Click!");
        //time,mesagetype,(data)
        //message types: 0 = set pulse length
        //               1 = set something else
        //model.send("00:00:00,0,1500,1500,1500,1500");

        //get values fom sliders
        //System.out.println("S1 progress:" + s1.getProgress() + " max:" + s1.getMax());

        float progress1 = (float)s1.getProgress() / (float)s1.getMax();
        float progress2 = (float)s2.getProgress() / (float)s2.getMax();
        float progress3 = (float)s3.getProgress() / (float)s3.getMax();
        float progress4 = (float)s4.getProgress() / (float)s4.getMax();

        float pulse1 = 1000 + progress1*1000;
        float pulse2 = 1000 + progress2*1000;
        float pulse3 = 1000 + progress3*1000;
        float pulse4 = 1000 + progress4*1000;

        model.send("00:00:00,0," + (int)pulse1 + ","+(int)pulse2+","+(int)pulse3+","+(int)pulse4);
    }

    @Override
    protected void onStop() {
        super.onStop();  // Always call the superclass method first
        model.closeConnection();
    }
}
