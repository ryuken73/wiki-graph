import React from 'react';
import { ForceGraph3D } from 'react-force-graph';

export default function Graph3D(props) {
  const {data} = props;
  const fgRef = React.useRef();
  return (
    <ForceGraph3D
      ref={fgRef}
      backgroundColor="#000003"
      graphData={data}
      nodeLabel="id"
      nodeAutoColorBy="group"
    />
  )
}
