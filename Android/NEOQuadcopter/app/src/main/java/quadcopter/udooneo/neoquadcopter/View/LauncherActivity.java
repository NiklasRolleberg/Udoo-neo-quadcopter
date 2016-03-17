package quadcopter.udooneo.neoquadcopter.View;

import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import quadcopter.udooneo.neoquadcopter.QuadcopterApplication;
import quadcopter.udooneo.neoquadcopter.R;
import quadcopter.udooneo.neoquadcopter.model.Model;

public class LauncherActivity extends AppCompatActivity implements View.OnClickListener {

    Button button;
    EditText ip_field;
    EditText port_field;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_launcher);

        button = (Button) findViewById(R.id.launcher_startbutton);
        ip_field = (EditText) findViewById(R.id.launcher_ipaddress);
        port_field = (EditText) findViewById(R.id.launcher_port);

        button.setOnClickListener(this);
    }

    @Override
    public void onClick(View v) {
        System.out.println("Click!");
        System.out.println("IP: " + ip_field.getText());
        System.out.println("port: " + port_field.getText());

        //Intent in = new Intent(getApplicationContext(),controlActivity.class);
        //startActivity(in);


        Model m = ((QuadcopterApplication) this.getApplication()).getModel();
        if(m.setupConnection(ip_field.getText().toString(), Integer.parseInt(port_field.getText().toString()))) {
            System.out.println("Start new Activity");
            Intent in = new Intent(getApplicationContext(),controlActivity.class);
            startActivity(in);
        }
        else {
            System.out.println("Failed!");
        }

    }
}
