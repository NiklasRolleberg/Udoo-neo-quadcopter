package quadcopter.udooneo.neoquadcopter.View;

import android.os.Bundle;
import android.support.design.widget.FloatingActionButton;
import android.support.design.widget.Snackbar;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.view.View;
import android.widget.Button;

import quadcopter.udooneo.neoquadcopter.QuadcopterApplication;
import quadcopter.udooneo.neoquadcopter.R;
import quadcopter.udooneo.neoquadcopter.model.Model;

public class controlActivity extends AppCompatActivity implements View.OnClickListener{

    Model model = null;
    int i = 0;

    Button sendmessage;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_control);

        model = ((QuadcopterApplication) this.getApplication()).getModel();

        sendmessage = (Button) findViewById(R.id.controlview_button);
        sendmessage.setOnClickListener(this);
    }

    @Override
    public void onClick(View v) {
        //System.out.println("Click!");
        model.send("Sent from android: " + (++i));
    }

    @Override
    protected void onStop() {
        super.onStop();  // Always call the superclass method first
        model.closeConnection();
    }
}
