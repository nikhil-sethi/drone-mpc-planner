let $ = require('jquery');
let d3 = require('d3');
var sqlite3 = require('sqlite3').verbose();
TimelinesChart = require('timelines-chart');
db_path = "./pats_records.db"

const system = localStorage.getItem("system");
const selected_hours_1 = localStorage.getItem("selected_hours_1").split(",");
const selected_hours_2 = localStorage.getItem("selected_hours_2").split(",");
$("#date_title").text("Statistics of "+ system)

//get data from date
promises = []
selected_hours_1.forEach(function(dt) {
    promise = getDataForDt(db_path, system, dt);
    promises.push(promise)
});

parsed_dates = [];
Promise.all(promises).then((values) => {
    parsed_dates = values.flat()
    histogram("#histogram_flight_duration", parsed_dates.map(elem => elem.flight_duration));

    // addHistogram("#histogram_flight_duration", parsed_dates.map(elem => elem.flight_duration))

    swim_data = getSwimPlotData(parsed_dates);
    swimPlot('swim_plot', swim_data)    
  
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

function swimPlot(plot_id, values){
    var format_date = d3.time.format("%Y-%m-%d %X");
    TimelinesChart()
        .data(values)
        .zQualitative(true)
        .maxLineHeight([35])
        .segmentTooltipContent(function(d) {
            return `<div>Info: ${d.data.RS_ID}, ${d.data.filename}</div>
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
        (document.getElementById(plot_id))
    
    $( ".legendG, .grp-axis, .y-axis, .group-tooltip, .line-tooltip" ).remove();
}

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
        height = 275 - margin.top - margin.bottom;
    
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
                filename : dt.filename,
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


// function addHistogram(plot_id, values){
    
//     //config
//     var vis = {
//         'container': '#rgb-luminance',
//     }
//     //calculated config
//     vis.svg = {
//         width: 420,
//         height: 275
//     }
//     vis.pad = {top: 80, right: 80, bottom: 80, left: 80}
//     vis.width = vis.svg.width - vis.pad.left - vis.pad.right
//     vis.height = vis.svg.height - vis.pad.top - vis.pad.bottom

//     var max = d3.max(values);
//     var min = d3.min(values);
//     var x = d3.scale.linear()
//         .domain([min, max])
//         .range([0, vis.width])
    
//     var hist1 = d3.layout.histogram()
//         .bins(x.ticks(20))(values);

//     var yMax = d3.max(hist1, function(d){return d.length});
//     var y = d3.scale.linear()
//         .domain([0, yMax])
//         .range([vis.height, 0])
    
//     var xAxis = d3.svg.axis()
//         .scale(x)
//         .orient('bottom')

//     var yAxis = d3.svg.axis()
//         .scale(y)
//         .ticks(4)
//         .tickFormat(function(d) { return d })
//         .orient('left')
    
//     var svg = d3.select(vis.container).append('svg')
//         .attr('width', vis.svg.width)
//         .attr('height', vis.svg.height)

//     var graph = svg.append('g')
//         .attr('class', 'graph')
//         .attr('width', vis.width)
//         .attr('height', vis.height)
//         .attr('transform', 'translate('+vis.pad.left+','+vis.pad.top+')')

    
//     var histogram = hist1//.concat(histG).concat(histB)
    
//     var bar = graph.selectAll('.bar')
//         .data(histogram)
//         .enter().append('g')
//         .attr('class', 'bar')
//         .attr('transform', function(d) { 
//             return 'translate('+x(d.x)+',0)'
//         })
    
//     bar.append('rect')
//         .attr('x', 1)
//         .attr('y', function(d) { return y(d.y) })
//         .attr('width', x(hist1[0].dx) - 1)
//         .attr('height', function(d) {
//         return vis.height - y(d.y)
//         })
//         .attr('fill', function(d) {
//         //map colors 0 - 150 (for legibility)
//         // var n = Math.round(d.x * 1.5)
//         return "red"
//         })
    
//     graph.append('g')
//         .attr('class', 'x axis')
//         .attr('transform', 'translate(0,' + vis.height + ')')
//         .call(xAxis)
//         // add axis label
//         .append('text')
//         .attr('class', 'label')
//         .attr('x', vis.width/2)
//         .attr('dy', svg.select('.x').node().getBBox().height + 14)
//         .text('Value    ')
    
//     graph.append('g')
//         .attr('class', 'y axis')
//         .call(yAxis)
//         // add axis label
//         .append('text')
//         .attr('class', 'label')
//         .attr('x', -vis.height/2)
//         .attr('dy', - svg.select('.y').node().getBBox().width)
//         .attr('transform', 'rotate(-90)')
//         .text('Frequence')
// }



// function addHistogram(plot_id, values){
    //helpers
    // function rgb2xyz (color) {
    //     color = [color[0]/255, color[1]/255, color[2]/255]
    //     var xyz = [0, 0, 0],
    //         matrix = [[0.412453, 0.357580, 0.180423],
    //                 [0.212671, 0.715160, 0.072169],
    //                 [0.019334, 0.119193, 0.950227]]
            
    //     matrix.forEach(function(row, i) {
    //     row.forEach(function(cell, j) {
    //         xyz[i] += color[j] * cell * 100
    //     })
    //     })
    
    //     return xyz
    // }
  
    // function xyz2Lab (color) {
    //     var d65 = [95.0456, 100, 108.8754],
    //         xyzN = [], 
    //         L, a, b
    //     //find X/Xn, Y/Yn, Z/Zn
    //     color.forEach(function(d, i) {
    //     xyzN.push(d/d65[i])
    //     })
    
    //     var f = function(t) {
    //     if (t > 0.008856)
    //         return Math.pow(t, 1/3)
    //     else
    //         return 7.787 * t + 16/116
    //     }
    
    //     if (xyzN[1] > 0.008856)
    //     L = 116 * Math.pow(xyzN[1], 1/3) - 16
    //     else
    //     L = 903.3 * xyzN[1]
    
    //     a = 500 * ( f(xyzN[0]) - f(xyzN[1]) )
    //     b = 200 * ( f(xyzN[1]) - f(xyzN[2]) )
    
    //     return [L, a, b]
    // }
    
    // function rgb2Lab(color) {
    //     return xyz2Lab(rgb2xyz(color))
    // }
    
    // function lumValsByPrime(prime) {
    //     var results = [],
    //         colors = { 
    //         r: function(r, i) { return [r, i, i] }, 
    //         g: function(g, i) { return [i, g, i] }, 
    //         b: function(b, i) { return [i, i, b] } 
    //         }
    
    //     prime = prime.toLowerCase()
    
    //     for (var p = 0; p < 256; p++) {
    //     for (var i = 0; i < 256; i++) {
    //         var luminance = Math.floor( rgb2Lab( colors[prime](p, i) )[0] )
    //         results.push(luminance)
    //     }
    //     }
    //     return results
    // }
    
    // //draw visualization
    
    // //config
    // var vis = {
    //     'container': '#rgb-luminance',
    // }
    // //calculated config
    // vis.svg = {
    //     width: 420,
    //     height: 275
    // }
    // vis.pad = {top: 80, right: 80, bottom: 80, left: 80}
    // vis.width = vis.svg.width - vis.pad.left - vis.pad.right
    // vis.height = vis.svg.height - vis.pad.top - vis.pad.bottom
    // vis.colors = { 
    //     r: function(r, i) { return 'rgb('+r+','+i+','+i+')' }, 
    //     g: function(g, i) { return 'rgb('+i+','+g+','+i+')' }, 
    //     b: function(b, i) { return 'rgb('+i+','+i+','+b+')' } 
    //     }
    
    // var x = d3.scale.linear()
    //     .domain([0, 10])
    //     .range([0, vis.width])
    
    // var histR = d3.layout.histogram()
    //     .bins(x.ticks(10))
    //     (values)
    // var histG = d3.layout.histogram()
    //     .bins(x.ticks(10))
    //     (values)
    // var histB = d3.layout.histogram()
    //     .bins(x.ticks(10))
    //     (values)
    
    // var y = d3.scale.linear()
    //     .domain([0, 20])
    //     .range([vis.height, 0])
    
    // var xAxis = d3.svg.axis()
    //     .scale(x)
    //     .orient('bottom')
    // var yAxis = d3.svg.axis()
    //     .scale(y)
    //     .ticks(4)
    //     .tickFormat(function(d) { return d })
    //     .orient('left')
    
    // var svg = d3.select(vis.container).append('svg')
    //     .attr('width', vis.svg.width)
    //     .attr('height', vis.svg.height)
    

    // var graph = svg.append('g')
    //     .attr('class', 'graph')
    //     .attr('width', vis.width)
    //     .attr('height', vis.height)
    //     .attr('transform', 'translate('+vis.pad.left+','+vis.pad.top+')')
    
    // //mark color, for fill function
    // histR.forEach(function(_, i) {
    //     histR[i].c = 'r'
    //     histG[i].c = 'g'
    //     histB[i].c = 'b'
    // })
    
    // var histogram = histR.concat(histG).concat(histB)
    
    // var bar = graph.selectAll('.bar')
    //     .data(histogram)
    //     .enter().append('g')
    //     .attr('class', 'bar')
    //     .attr('transform', function(d) { 
    //         return 'translate('+x(d.x)+',0)'
    //     })
    
    // bar.append('rect')
    //     .attr('x', 1)
    //     .attr('y', function(d) { return y(d.y) })
    //     .attr('width', x(histR[0].dx) - 1)
    //     .attr('height', function(d) {
    //     return vis.height - y(d.y)
    //     })
    //     .attr('fill', function(d) {
    //     //map colors 0 - 150 (for legibility)
    //     var n = Math.round(d.x * 1.5)
    //     return vis.colors[d.c](255, n)//'rgb(255,'+n+','+n+')'
    //     })
    
    // graph.append('g')
    //     .attr('class', 'x axis')
    //     .attr('transform', 'translate(0,' + vis.height + ')')
    //     .call(xAxis)
    //     // add axis label
    //     .append('text')
    //     .attr('class', 'label')
    //     .attr('x', vis.width/2)
    //     .attr('dy', svg.select('.x').node().getBBox().height + 14)
    //     .text('Luminance Value')
    
    // graph.append('g')
    //     .attr('class', 'y axis')
    //     .call(yAxis)
    //     // add axis label
    //     .append('text')
    //     .attr('class', 'label')
    //     .attr('x', -vis.height/2)
    //     .attr('dy', - svg.select('.y').node().getBBox().width)
    //     .attr('transform', 'rotate(-90)')
    //     .text('Thousands of Colors')
    
    
    // //hide colors on button click
    // var visNode = d3.select('#rgb-luminance'),
    //     rects = visNode.selectAll('rect'),
    //     btns = ['rgb', 'red', 'green', 'blue']
    
    // btns.forEach(function(btnColor) {
    //     visNode.select('.'+btnColor).on('click', function() {
    //     rects.transition()
    //         .duration(500)
    //         .attr('height', function(d) {
    //         if (d.c !== btnColor.substr(0,1) && btnColor !== 'rgb')
    //             return 0
    //         else
    //             return vis.height - y(d.y)
    //         })
    //         .attr('y', function(d) {
    //         if (d.c !== btnColor.substr(0,1) && btnColor !== 'rgb')
    //             return vis.height
    //         else
    //             return y(d.y)
    //         })
    //     })
    // })
// }
