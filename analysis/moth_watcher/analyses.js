let $ = require('jquery');
let d3 = require('d3');
var sqlite3 = require('sqlite3').verbose();
TimelinesChart = require('timelines-chart');
db_path = "./moth_records.db"

const system = localStorage.getItem("system");
const selected_hours = localStorage.getItem("selected_hours").split(",");
$("#date_title").text("Statistics of "+ system)

//get data from date
promises = []
selected_hours.forEach(function(dt) {
    promise = getDataForDt(db_path, system, dt);
    promises.push(promise)
});

parsed_dates = [];
Promise.all(promises).then((values) => {
    parsed_dates = values.flat()
    histogram("#histogram_flight_duration", parsed_dates.map(elem => elem.flight_duration));
    swim_data = getSwimPlotData(parsed_dates);
    var format_date = d3.time.format("%Y-%m-%d %X");
    TimelinesChart()
        .data(swim_data)
        .zQualitative(true)
        .maxLineHeight([35])
        .segmentTooltipContent(function(d) {
            return `<div>RS_ID : ${d.data.RS_ID}</div>
            <div>Start : ${format_date(d.timeRange[0])}</div>
            <div>Duration : ${parseInt(d.data.duration*1000)} ms</div>
            <table style="width:100%">
            <tr>
              <th>Variable</th>
              <th>Mean</th>
              <th>Std</th>
              <th>Max</th>
            </tr>
            <tr>
              <td>Velocity</td>
              <td>${d.data.velocity_mean.toFixed(3)}</td>
              <td>${d.data.velocity_std.toFixed(3)}</td>
              <td>${d.data.velocity_max.toFixed(3)}</td>
            </tr>
            <tr>
              <td>Turning angle</td>
              <td>${d.data.turning_angle_mean.toFixed(3)}</td>
              <td>${d.data.turning_angle_std.toFixed(3)}</td>
              <td>${d.data.turning_angle_max.toFixed(3)}</td>
            </tr>
            <tr>
              <td>Radial acceleration</td>
              <td>${d.data.radial_accelaration_mean.toFixed(3)}</td>
              <td>${d.data.radial_accelaration_std.toFixed(3)}</td>
              <td>${d.data.radial_accelaration_max.toFixed(3)}</td>
            </tr>
          </table>`
        })
        (document.getElementById('swim_plot'))


    
    $( ".legendG, .grp-axis, .y-axis, .group-tooltip, .line-tooltip" ).remove();
  
    histogram("#histogram_average_velocity_mean", parsed_dates.map(elem => elem.velocity_mean));
    histogram("#histogram_average_velocity_std", parsed_dates.map(elem => elem.velocity_std));
    histogram("#histogram_average_velocity_max", parsed_dates.map(elem => elem.velocity_max));

    histogram("#histogram_average_angle_mean", parsed_dates.map(elem => elem.turning_angle_mean));
    histogram("#histogram_average_angle_std", parsed_dates.map(elem => elem.turning_angle_std));
    histogram("#histogram_average_angle_max", parsed_dates.map(elem => elem.turning_angle_max));

    histogram("#histogram_average_acceleration_mean", parsed_dates.map(elem => elem.radial_accelaration_mean));
    histogram("#histogram_average_acceleration_std", parsed_dates.map(elem => elem.radial_accelaration_std));
    histogram("#histogram_average_acceleration_max", parsed_dates.map(elem => elem.radial_accelaration_max));
});

function getDataForDt(db_path, system, date){
    var parse_date = d3.time.format("%d/%m/%Y_%H").parse;
    var format_date = d3.time.format("%Y-%m-%dT%X%Z");
    var min_date = parse_date(date);
    var max_date = addHour.call(min_date);

    min_date = format_date(min_date);
    max_date = format_date(max_date);

    return new Promise((resolve, reject) => {
        const db = new sqlite3.Database(db_path);
        const dt_data = [];
        var stat = `SELECT * from analytic_records where system = '${system}' and flight_time between '${min_date}' and '${max_date}'`
        db.each(stat, (err, row) => {
            if (err) {
                reject(err); // optional: you might choose to swallow errors.
            } else {
                dt_data.push(row); // accumulate the data
            }
        }, (err, n) => {
            if (err) {
                reject(err); // optional: again, you might choose to swallow this error.
            } else {
                resolve(dt_data); // resolve the promise
            }
        });
    });
}

function addHour() {
    var date = new Date(this.valueOf());
    date.setHours(date.getHours() + 1);
    return date;
};

function histogram(graph_id, values){

    var color = "steelblue";
    
    // A formatter for counts.
    var formatCount = d3.format(",.0f");
    
    var margin = {top: 10, right: 20, bottom: 20, left: 20},
        width = 420 - margin.left - margin.right,
        height = 250 - margin.top - margin.bottom;
    
    var max = d3.max(values);
    var min = d3.min(values);
    var x = d3.scale.linear()
          .domain([min, max])
          .range([0, width]);
    
    // Generate a histogram using twenty uniformly-spaced bins.
    var data = d3.layout.histogram()
        .bins(x.ticks(20))(values);
    
    var yMax = d3.max(data, function(d){return d.length});
    var yMin = d3.min(data, function(d){return d.length});
    var colorScale = d3.scale.linear()
                .domain([yMin, yMax])
                .range([d3.rgb(color).brighter(), d3.rgb(color).darker()]);
    
    var y = d3.scale.linear()
        .domain([0, yMax])
        .range([height, 0]);
    
    var xAxis = d3.svg.axis()
        .scale(x)
        .orient("bottom");
    
    var svg = d3.select(graph_id).append("svg")
        .attr("width", width + margin.left + margin.right)
        .attr("height", height + margin.top + margin.bottom)
      .append("g")
        .attr("transform", "translate(" + margin.left + "," + margin.top + ")");
    
    var bar = svg.selectAll(".bar")
        .data(data)
      .enter().append("g")
        .attr("class", "bar")
        .attr("transform", function(d) { return "translate(" + x(d.x) + "," + y(d.y) + ")"; });
    
    bar.append("rect")
        .attr("x", 1)
        .attr("width", (x(data[0].dx) - x(0)) - 1)
        .attr("height", function(d) { return height - y(d.y); })
        .attr("fill", function(d) { return colorScale(d.y) });
    
    bar.append("text")
        .attr("dy", ".75em")
        .attr("y", -12)
        .attr("x", (x(data[0].dx) - x(0)) / 2)
        .attr("text-anchor", "middle")
        .text(function(d) { return formatCount(d.y); });
    
    svg.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(0," + height + ")")
        .call(xAxis);
}    

function getSwimPlotData(data){
    return [{
        group: "",
        data: getGroupData(data)
    }]

    function getGroupData(data) {
        return [{
            label: '',
            data: getSegmentsData(data)
        }]
    }

    function getSegmentsData(data){
        var parse_date;
        lane = [];
        data.forEach(function(dt) {
            parse_date = d3.time.format("%Y-%m-%dT%H:%M:%S").parse;
            start = parse_date(dt.flight_time.split("+")[0]);
            end = start;
            end.setSeconds(start.getSeconds() + dt.flight_duration);
            flight_data = {
                timeRange : [start, end],
                duration : dt.flight_duration,
                val : "flight",
                RS_ID : dt.start_RS_ID,
                velocity_mean : dt.velocity_mean,
                velocity_std : dt.velocity_std,
                velocity_max : dt.velocity_max,
                turning_angle_mean: dt.turning_angle_mean,
                turning_angle_std: dt.turning_angle_std,
                turning_angle_max: dt.turning_angle_max,
                radial_accelaration_mean : dt.radial_accelaration_mean,
                radial_accelaration_std : dt.radial_accelaration_std,
                radial_accelaration_max : dt.radial_accelaration_max
            }
            lane.push(flight_data)
        })
        return lane;
    }
}