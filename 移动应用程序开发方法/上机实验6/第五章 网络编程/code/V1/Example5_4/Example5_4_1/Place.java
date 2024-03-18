package edu.zjut.example.example5_2_1;

/**
 * Created by Administrator on 2016/3/9.
 */
public class Place {
    private String name;
    private double lng;
    private double lat;
    private String address;
    private String telephone;

    public void setName(String name) {
        this.name = name;
    }

    public void setLng(double lng) {
        this.lng = lng;
    }

    public void setLat(double lat) {
        this.lat = lat;
    }

    public void setAddress(String address) {
        this.address = address;
    }

    public void setTelephone(String telephone) {
        this.telephone = telephone;
    }

    public String getName() {
        return name;
    }

    public double getLng() {
        return lng;
    }

    public double getLat() {
        return lat;
    }

    public String getAddress() {
        return address;
    }

    public String getTelephone() {
        return telephone;
    }

    public String toString() {
        String result = name + ";(" + lng + "," + lat + ")," + address + "," + telephone;
        return result;
    }
}
