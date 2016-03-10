package quadcopter.udooneo.neoquadcopter;

import android.app.Application;

import quadcopter.udooneo.neoquadcopter.model.Model;

/**
 * Created by Niklas on 2016-03-10.
 */
public class QuadcopterApplication extends Application {

    private Model model = new Model();

    public Model getModel() {
        return this.model;
    }
}
